#include <FrontEnd.h>
#include <MemoryUtil.h>
#include <arch/board/board.h>

#include <MP.h>

#define SAMPLE_SIZE (1024)
//#define DEBUG_ENABLE
FrontEnd *theFrontEnd;

const int32_t channel_num = AS_CHANNEL_MONO;
const int32_t bit_length  = AS_BITLENGTH_16;
const int32_t sample_size = SAMPLE_SIZE;
const int32_t frame_size  = sample_size * (bit_length / 8) * channel_num;
bool isErr = false;

//******/
#define ARM_MATH_CM4
#define __FPU_PRESENT 1U
#include <arm_math.h>
arm_rfft_fast_instance_f32 S;
//******/


#define IDLE_STATE     (0)
#define STARTBIT_STATE (1)
#define BITREC_STATE   (2)
#define STOPBIT_STATE  (3)
#define FETCH_INTERVAL (4)
#define MSBBIT_INDEX   (7)

const int SPACE = 38000;
const int MARK  = 40000;

static uint8_t frame_cnt = 0;
static uint8_t fetch_timing = 1; 
static uint8_t bpos = 0;
static uint8_t cur_state = IDLE_STATE;
static char    output = 0;

void idle_phase(uint8_t sbit) {
  if (sbit == 0) {
    cur_state = STARTBIT_STATE;
  }
  
  frame_cnt = 0;
  fetch_timing = 1;
  output = 0;
  return;
}

void startbit_phase(uint8_t sbit) {
  ++frame_cnt;
  if (frame_cnt != fetch_timing) return;
  debug_print(sbit);
  
  cur_state = BITREC_STATE;
  fetch_timing += FETCH_INTERVAL;
  return;
}

void bitrec_phase(uint8_t sbit) {
  ++frame_cnt;
  if (frame_cnt != fetch_timing) return;
  debug_print(sbit);  
  
  output = output | (sbit << bpos);
  fetch_timing += FETCH_INTERVAL;
  if (++bpos > MSBBIT_INDEX) {
    cur_state = STOPBIT_STATE;
  }
  return;
}


bool stopbit_phase(uint8_t sbit) {
  ++frame_cnt;
  if (frame_cnt != fetch_timing) return;
  debug_print(sbit);
  
  Serial.write(output);  // interim implementation
  frame_cnt = 0;
  bpos = 0;
  cur_state = IDLE_STATE;
  return;
}


void frontend_attention_cb(const ErrorAttentionParam *param) {
  Serial.println("ERROR: Attention! Something happened at FrontEnd");
  if (param->error_code >= AS_ATTENTION_CODE_WARNING) isErr = true;
}


static bool frontend_done_cb(AsMicFrontendEvent ev, uint32_t result, uint32_t detail){
  UNUSED(ev);  UNUSED(result);  UNUSED(detail);  
  return true;
}

static void frontend_pcm_cb(AsPcmDataParam pcm) {
  static uint8_t mono_input[frame_size];
  static uint8_t stereo_output[frame_size*2];  
  static const bool time_measurement = false;
  if (time_measurement) {
    static uint32_t last_time = 0;
    uint32_t current_time = micros();
    uint32_t duration = current_time - last_time;
    last_time = current_time;
    Serial.println("duration = " + String(duration));
  }

  frontend_signal_input(pcm, mono_input, frame_size);
  signal_process((int16_t*)mono_input, (int16_t*)stereo_output, sample_size);
  return;
}

void frontend_signal_input(AsPcmDataParam pcm, uint8_t* input, uint32_t frame_size) {
  /* clean up the input buffer */
  memset(input, 0, frame_size);

  if (!pcm.is_valid) {
    Serial.println("WARNING: Invalid data! @frontend_signal_input");
    return;
  }
   
  if (pcm.size > frame_size) {
    Serial.print("WARNING: Captured size is too big! -");
    Serial.print(String(pcm.size));
    Serial.println("- @frontend_signal_input");
    pcm.size = frame_size;
  } 
  
  /* copy the signal to signal_input buffer */
  if (pcm.size != 0) {
    memcpy(input, pcm.mh.getPa(), pcm.size);
  } else {
    Serial.println("WARNING: Captured size is zero! @frontend_signal_input");
  }  
}

void signal_process(int16_t* mono_input, int16_t* stereo_output, uint32_t sample_size) {
  uint32_t start_time = micros();


  static float pSrc[SAMPLE_SIZE];
  static float pDst[SAMPLE_SIZE];
  static float tmpBuf[SAMPLE_SIZE];
  float maxValue;
  uint32_t index;
  const float df = AS_SAMPLINGRATE_192000/SAMPLE_SIZE;  

  arm_q15_to_float(&mono_input[0], &pSrc[0], SAMPLE_SIZE);
  arm_rfft_fast_f32(&S, &pSrc[0], &tmpBuf[0], 0);
  arm_cmplx_mag_f32(&tmpBuf[0], &pDst[0], SAMPLE_SIZE / 2);
  arm_max_f32(&pDst[0], SAMPLE_SIZE/2, &maxValue, &index);

  float peakFs = (float)index*df;
  const int fc = ((SPACE + MARK)/2) / df; // 39kHz

  uint8_t sbit;
  if (index < fc) sbit = 0;
  else if (index > fc) sbit = 1;
  
  switch(cur_state) {
   case IDLE_STATE:   idle_phase(sbit); break;
   case STARTBIT_STATE:  startbit_phase(sbit); break;
   case BITREC_STATE: bitrec_phase(sbit); break;
   case STOPBIT_STATE:   stopbit_phase(sbit); break;
  }

  uint32_t duration = micros() - start_time;
  //Serial.println("process time = " + String(duration));
  return;
}


void setup() {
  const int subcore = 1;
  Serial.begin(115200);
  MP.begin(subcore);
  arm_rfft_fast_init_f32(&S, SAMPLE_SIZE);
  
  /* Initialize memory pools and message libs */
  initMemoryPools();
  createStaticPools(MEM_LAYOUT_RECORDINGPLAYER);

  /* setup FrontEnd and Mixer */
  theFrontEnd = FrontEnd::getInstance();
  
  /* set clock mode */
  theFrontEnd->setCapturingClkMode(FRONTEND_CAPCLK_HIRESO);

  /* begin FrontEnd and OuputMixer */
  theFrontEnd->begin(frontend_attention_cb);
  Serial.println("Setup: FrontEnd began");

  /* activate FrontEnd and Mixer */
  theFrontEnd->setMicGain(0);
  theFrontEnd->activate(frontend_done_cb);
  delay(100); /* waiting for Mic startup */
  Serial.println("Setup: FrontEnd activated");

  /* Initialize FrontEnd */
  AsDataDest dst;
  dst.cb = frontend_pcm_cb;
  theFrontEnd->init(channel_num, bit_length, sample_size, AsDataPathCallback, dst);
  Serial.println("Setup: FrontEnd initialized");

  /* Unmute */
  theFrontEnd->start();
  Serial.println("Setup: FrontEnd started");
}

void loop() {
  if (isErr == true) {
    board_external_amp_mute_control(true); 
    theFrontEnd->stop();
    theFrontEnd->deactivate();
    theFrontEnd->end();
    Serial.println("Capturing Process Terminated");
    while(1) {};
  }
}


void debug_print(uint8_t sbit) {
#ifdef DEBUG_ENABLE
  static bool first_print = true;
  if (first_print) {
    Serial.println("state, sbit, bpos, fcnt");
    first_print = false;   
  }
  Serial.print(String(cur_state));
  Serial.print("," + String(sbit));
  Serial.print("," + String(bpos));
  Serial.print("," + String(frame_cnt));
  Serial.println();  
#endif
}
