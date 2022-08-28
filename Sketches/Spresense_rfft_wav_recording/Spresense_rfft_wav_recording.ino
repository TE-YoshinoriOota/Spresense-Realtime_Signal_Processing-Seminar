#include <FrontEnd.h>
#include <OutputMixer.h>
#include <MemoryUtil.h>
#include <arch/board/board.h>
#include <SDHCI.h>
#include <MP.h>

#define SAMPLE_SIZE (1024)

FrontEnd *theFrontEnd;
OutputMixer *theMixer;
bool isErr = false;

//***/
#include <audio/utilities/wav_containerformat.h>
#include <audio/utilities/wav_containerformat_parser.h>
#define WAV_FILE "test.wav"
WAVHEADER wav_format;
SDClass SD;
File myFile;
uint32_t data_size = 0;
bool b_recording = true;

#define ARM_MATH_CM4
#define __FPU_PRESENT 1U
#include <cmsis/arm_math.h>
arm_rfft_fast_instance_f32 S;
//***/

/* recording settings */
const int32_t channel_num = AS_CHANNEL_MONO;
const int32_t bit_length  = AS_BITLENGTH_16;
const int32_t sample_size = SAMPLE_SIZE;
const int32_t frame_size  = sample_size * (bit_length / 8) * channel_num;

uint8_t input[frame_size];
uint8_t stereo_output[frame_size*2];

static void outputmixer0_send_cb(int32_t identifier, bool is_end) {
  UNUSED(identifier);  UNUSED(is_end);
  return;
}

static void frontend_pcm_cb(AsPcmDataParam pcm) {
  static const uint32_t recording_time = 10000; // milli sec 
  static uint32_t start_time = millis();

  /* clean up the input buffer */
  memset(&input[0], 0, frame_size);
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
    memcpy(&input[0], pcm.mh.getPa(), pcm.size);
  } else {
    Serial.println("WARNING: Captured size is zero! @frontend_signal_input");
  }  

//***/
  static float pTmp[SAMPLE_SIZE];
  static float p1[SAMPLE_SIZE];
  static float p2[SAMPLE_SIZE];
  q15_t* q15_mono = (q15_t*)input;
  
  arm_q15_to_float(&q15_mono[0], &pTmp[0], SAMPLE_SIZE);
  arm_rfft_fast_f32(&S, &pTmp[0], &p1[0], 0);
  int pitch_shift = 20;
  memcpy(&p2[pitch_shift*2], &p1[0], (SAMPLE_SIZE/2-pitch_shift*2)*sizeof(float));
  arm_rfft_fast_f32(&S, &p2[0], &pTmp[0], 1);
  arm_float_to_q15(&pTmp[0], &q15_mono[0], SAMPLE_SIZE);
//***/

  memset(&stereo_output[0], 0, frame_size*2);
  int16_t* inp = (int16_t*)input;
  int16_t* tmp = (int16_t*)stereo_output;
  for (int n = 0; n < sample_size; ++n) {
    tmp[n*2] = tmp[n*2+1] = inp[n]; 
  }

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
  pcm_param.sample = sample_size*2;
  pcm_param.is_valid = true;
  memcpy(pcm_param.mh.getPa(), stereo_output, pcm_param.size);
  
  int err = theMixer->sendData(OutputMixer0, outputmixer0_send_cb, pcm_param);
  if (err != OUTPUTMIXER_ECODE_OK) {
    Serial.println("ERROR: sendData -" + String(err) + "- @mixer_stereo_output");
    isErr = true;
  }

  if (b_recording) {
    data_size += frame_size;
    //Serial.println(data_size);
    myFile.write((uint8_t*)&input[0], frame_size);
    if ((millis() - start_time) > recording_time) {
      myFile.seek(0);
      wav_format.total_size = data_size + sizeof(WAVHEADER) - 8;
      wav_format.data_size  = data_size;
      int ret = myFile.write((uint8_t*)&wav_format, sizeof(WAVHEADER));
      if (ret != sizeof(WAVHEADER))  {
        Serial.println("Fail to write file(wav header)");
      }
      Serial.println("recording finished!");
      myFile.close();
      b_recording = false;
    }
  }  
  return;
}


void setup() {
  Serial.begin(115200);
  while(!SD.begin()){ Serial.println("Insert SD Card");};

  if (SD.exists(WAV_FILE)) SD.remove(WAV_FILE);
  myFile = SD.open(WAV_FILE, FILE_WRITE);
  if (myFile == NULL) {
    Serial.println("File open failed");
    exit(1);
  }

  // Write WAV header
  wav_format.riff     = CHUNKID_RIFF;
  wav_format.wave     = FORMAT_WAVE;
  wav_format.fmt      = SUBCHUNKID_FMT;
  wav_format.fmt_size = FMT_CHUNK_SIZE;
  wav_format.format   = FORMAT_ID_PCM;
  wav_format.channel  = channel_num;
  wav_format.rate     = AS_SAMPLINGRATE_48000;
  wav_format.avgbyte  = AS_SAMPLINGRATE_48000 * channel_num * (bit_length / 8);
  wav_format.block    = channel_num * (bit_length / 8);
  wav_format.bit      = bit_length;
  wav_format.data     = SUBCHUNKID_DATA; 
  wav_format.total_size = data_size + sizeof(WAVHEADER) - 8;
  wav_format.data_size  = data_size;
  int ret = myFile.write((uint8_t*)&wav_format, sizeof(WAVHEADER));
  if (ret != sizeof(WAVHEADER))  {
    Serial.println("Fail to write file(wav header)");
    myFile.close();
    exit(1);
  }

  //***/
  arm_rfft_fast_init_f32(&S, SAMPLE_SIZE);
  //***/ 

  /* Initialize memory pools and message libs */
  initMemoryPools();
  createStaticPools(MEM_LAYOUT_RECORDINGPLAYER);
 
  /* setup FrontEnd and Mixer */
  theFrontEnd = FrontEnd::getInstance();
  theMixer = OutputMixer::getInstance();

  theFrontEnd->setCapturingClkMode(FRONTEND_CAPCLK_NORMAL);
  theMixer->setRenderingClkMode(OUTPUTMIXER_RNDCLK_NORMAL); 

  /* begin FrontEnd and OuputMixer */
  theFrontEnd->begin(NULL);
  theMixer->begin();
  Serial.println("Setup: FrontEnd and OutputMixer began");

  /* activate FrontEnd and Mixer */
  theFrontEnd->activate(NULL);
  theMixer->create(NULL);
  theMixer->activate(OutputMixer0, NULL);
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
