#include <FrontEnd.h>
#include <OutputMixer.h>
#include <MemoryUtil.h>
#include <arch/board/board.h>

#define SAMPLE_SIZE (720)

//*****
#define ARM_MATH_CM4
#define __FPU_PRESENT 1U
#include <cmsis/arm_math.h>
#define TAPS 255
arm_fir_instance_f32 S;
float pCoeffs[TAPS];
float pState[TAPS+SAMPLE_SIZE-1];

#define VOLUME_STEP (256)
//*****

FrontEnd *theFrontEnd;
OutputMixer *theMixer;

const int32_t channel_num = AS_CHANNEL_MONO;
const int32_t bit_length  = AS_BITLENGTH_16;
const int32_t sample_size = SAMPLE_SIZE;
const int32_t frame_size  = sample_size * (bit_length / 8) * channel_num;
bool isErr = false;

void frontend_attention_cb(const ErrorAttentionParam *param) {
  Serial.println("ERROR: Attention! Something happened at FrontEnd");
  if (param->error_code >= AS_ATTENTION_CODE_WARNING) isErr = true;
}

void mixer_attention_cb(const ErrorAttentionParam *param){
  Serial.println("ERROR: Attention! Something happened at Mixer");
  if (param->error_code >= AS_ATTENTION_CODE_WARNING) isErr = true;
}

static bool frontend_done_cb(AsMicFrontendEvent ev, uint32_t result, uint32_t detail){
  UNUSED(ev);  UNUSED(result);  UNUSED(detail);  
  return true;
}

static void outputmixer_done_cb(MsgQueId requester_dtq, MsgType reply_of, AsOutputMixDoneParam* done_param) {
  UNUSED(requester_dtq);  UNUSED(reply_of);  UNUSED(done_param);
  return;
}

static void outputmixer0_send_cb(int32_t identifier, bool is_end) {
  UNUSED(identifier);  UNUSED(is_end);
  return;
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
  mixer_stereo_output(stereo_output, frame_size);
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

  /* memory pool for 1.5sec (= 720*100*(1/48000)) */
  static const int lines = 100;
  static int16_t src_buf[SAMPLE_SIZE*lines];  /* 2*720*100=144kBytes */
  
  /* shift the buffer data in src_buf and add the latest data to top of the bufer */
  memcpy(&src_buf[0], &src_buf[SAMPLE_SIZE], SAMPLE_SIZE*sizeof(int16_t)*(lines-1));
  memcpy(&src_buf[(lines-1)*SAMPLE_SIZE], &mono_input[0], SAMPLE_SIZE*sizeof(int16_t));
  
  /* set constatns for echo effect */
  static const uint32_t D1_in_ms = 300;
  /* milli sec */
  static const uint32_t D2_in_ms = 600; 
  /* milli sec */
  static const uint32_t offset1 = D1_in_ms * 48000 / 1000;
  static const uint32_t offset2 = D2_in_ms * 48000 / 1000;
  const int src_buf_end_point = lines*SAMPLE_SIZE-1;
  for (int n = SAMPLE_SIZE-1; n >= 0; --n) {
    /* set h1 = 1/2, h2 = 1/4 to reduce calculation costs */
    mono_input[(SAMPLE_SIZE-1)-n] = src_buf[src_buf_end_point-n]
                                  + src_buf[src_buf_end_point-n-offset1]/2 
                                  + src_buf[src_buf_end_point-n-offset2]/4;
  }

  /* clean up the output buffer */
  memset(stereo_output, 0, sizeof(int16_t)*sample_size*2);
  
  /* copy the signal to output buffer */
  for (int n = SAMPLE_SIZE-1; n >= 0; --n)  {
    stereo_output[n*2] = stereo_output[n*2+1] = mono_input[n];
  }

  uint32_t duration = micros() - start_time;
  //Serial.println("process time = " + String(duration));
  return;
}

void mixer_stereo_output(uint8_t* stereo_output, uint32_t frame_size) {
  
  /* Alloc MemHandle */
  AsPcmDataParam pcm_param;
  if (pcm_param.mh.allocSeg(S0_REND_PCM_BUF_POOL, frame_size) != ERR_OK) {
    Serial.println("ERROR: Cannot allocate memory @mixer_stereo_output");
    isErr = false;
    return;
  }
  
  /* Set PCM parameters */
  pcm_param.is_end = false;
  pcm_param.identifier = OutputMixer0;
  pcm_param.callback = 0;
  pcm_param.bit_length = bit_length;
  pcm_param.size = frame_size*2;
  pcm_param.sample = frame_size;
  pcm_param.is_valid = true;
  memcpy(pcm_param.mh.getPa(), stereo_output, pcm_param.size);
 
  int err = theMixer->sendData(OutputMixer0, outputmixer0_send_cb, pcm_param);
  if (err != OUTPUTMIXER_ECODE_OK) {
    Serial.println("ERROR: sendData -" + String(err) + "- @mixer_stereo_output");
    isErr = true;
  }
}

//***/
void setupFirBPF(int high, int low) {
  static int high_ = 0;
  static int low_  = VOLUME_STEP-1;
  if ((high_ == high) && (low_ == low)) return;
  high_ = high; low_ = low;

  const uint32_t freq_step = AS_SAMPLINGRATE_48000/2/VOLUME_STEP;
  uint32_t CUTTOFF_LOW_FREQ_HZ = low_*freq_step; 
  uint32_t CUTTOFF_HIGH_FREQ_HZ = high_*freq_step; 
  float Fl = (float)CUTTOFF_LOW_FREQ_HZ/AS_SAMPLINGRATE_48000;
  float Fh = (float)CUTTOFF_HIGH_FREQ_HZ/AS_SAMPLINGRATE_48000;
  const int H_TAPS = TAPS/2;
  int n = 0;
  for (int k = H_TAPS; k >= -H_TAPS; --k) {
    if (k == 0) pCoeffs[n] = 2.*(Fh - Fl);
    else {
      pCoeffs[n] = 2.*Fh*arm_sin_f32(2.*PI*Fh*k)/(2.*PI*Fh*k) 
                 - 2.*Fl*arm_sin_f32(2.*PI*Fl*k)/(2.*PI*Fl*k);
    }
    ++n;
  }

  for (int m = 0; m < TAPS; ++m) {
    pCoeffs[m] = (0.5 - 0.5*arm_cos_f32(2*PI*m/TAPS))*pCoeffs[m];
  }

  arm_fir_init_f32(&S, TAPS, pCoeffs, pState, SAMPLE_SIZE);
  Serial.println("BPF Changed : " + String(CUTTOFF_LOW_FREQ_HZ) + " - " + String(CUTTOFF_HIGH_FREQ_HZ)); 
}
//***/


void setup() {
  Serial.begin(115200);

  /* Initialize memory pools and message libs */
  initMemoryPools();
  createStaticPools(MEM_LAYOUT_RECORDINGPLAYER);

  /* setup FrontEnd and Mixer */
  theFrontEnd = FrontEnd::getInstance();
  theMixer = OutputMixer::getInstance();
  
  /* set clock mode */
  theFrontEnd->setCapturingClkMode(FRONTEND_CAPCLK_NORMAL);

  /* begin FrontEnd and OuputMixer */
  theFrontEnd->begin(frontend_attention_cb);
  theMixer->begin();
  Serial.println("Setup: FrontEnd and OutputMixer began");

  /* activate FrontEnd and Mixer */
  theFrontEnd->setMicGain(0);
  theFrontEnd->activate(frontend_done_cb);
  theMixer->create(mixer_attention_cb);
  theMixer->activate(OutputMixer0, outputmixer_done_cb);
  delay(100); /* waiting for Mic startup */
  Serial.println("Setup: FrontEnd and OutputMixer activated");

  /* Initialize FrontEnd */
  AsDataDest dst;
  dst.cb = frontend_pcm_cb;
  theFrontEnd->init(channel_num, bit_length, sample_size, AsDataPathCallback, dst);
  Serial.println("Setup: FrontEnd initialized");

  /* Set rendering volume */
  theMixer->setVolume(-10, -10, -10); /* -10dB */

  /* Unmute */
  board_external_amp_mute_control(false);
  theFrontEnd->start();
  Serial.println("Setup: FrontEnd started");
}

void loop() {
  if (isErr == true) {
    board_external_amp_mute_control(true); 
    theFrontEnd->stop();
    theFrontEnd->deactivate();
    theMixer->deactivate(OutputMixer0);
    theFrontEnd->end();
    theMixer->end();
    Serial.println("Capturing Process Terminated");
    while(1) {};
  }
}
