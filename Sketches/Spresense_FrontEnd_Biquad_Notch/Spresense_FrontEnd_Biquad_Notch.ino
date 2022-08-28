#include <FrontEnd.h>
#include <OutputMixer.h>
#include <MemoryUtil.h>
#include <arch/board/board.h>

#define SAMPLE_SIZE (720)

//*****
#define ARM_MATH_CM4
#define __FPU_PRESENT 1U
#include <cmsis/arm_math.h>
arm_biquad_cascade_df2T_instance_f32 S;
const int numStages = 1;
float pCoeffs[5*numStages];
float pState[2*numStages];
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

//***/
  static float pSrc[SAMPLE_SIZE];
  static float pDst[SAMPLE_SIZE];
  q15_t* q15_mono = (q15_t*)mono_input;
  arm_q15_to_float(&q15_mono[0], &pSrc[0], SAMPLE_SIZE);
  arm_biquad_cascade_df2T_f32(&S, &pSrc[0], &pDst[0], SAMPLE_SIZE); // 2000Hz LPF
  arm_float_to_q15(&pDst[0], &q15_mono[0], SAMPLE_SIZE);
  mono_input = (int16_t*)q15_mono;
//***/

  /* clean up the output buffer */
  memset(stereo_output, 0, sizeof(int16_t)*sample_size*2);
  /* copy the signal to output buffer */
  for (int n = SAMPLE_SIZE-1; n >= 0; --n)  {
    stereo_output[n*2] = stereo_output[n*2+1] = mono_input[n];
  }

  uint32_t duration = micros() - start_time;
  Serial.println("process time = " + String(duration));
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
void initializeBiquadLPF() {
  const uint32_t CUTTOFF_FREQ_HZ = 1500; 
  float Wc = 2.*PI*CUTTOFF_FREQ_HZ/AS_SAMPLINGRATE_48000;
  float Octave = 1./10.;
  float Bandwidth = (pow(2., Octave) - 1.)/pow(2., Octave/2);
  float Alpha = sin(Wc)*sinh(log(2.)/2.0*Bandwidth*Wc/sin(Wc));  

  float numerator = 1.+arm_cos_f32(Wc);
  float b10 = 1.;
  float b11 = -2.*arm_cos_f32(Wc);
  float b12 = 1.;

  float a10 = 1. + Alpha;
  float a11 = -2.*arm_cos_f32(Wc);
  float a12 = 1. - Alpha;

  /* normalized by a0 */
  pCoeffs[0] =  b10/a10;
  pCoeffs[1] =  b11/a10;
  pCoeffs[2] =  b12/a10;
  pCoeffs[3] = -a11/a10;
  pCoeffs[4] = -a12/a10;

  arm_biquad_cascade_df2T_init_f32(&S, numStages, &pCoeffs[0], &pState[0]);
}
//***/


void setup() {
  Serial.begin(115200);
  //***
  initializeBiquadLPF();  
  //***  

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
