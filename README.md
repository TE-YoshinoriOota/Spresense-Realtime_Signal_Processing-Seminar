# Spresense-Realtime_Signal_Processing-Seminar
Spresense Realtime Signal Processing Seminar materials

## Seminar material
[Get started Real-time Signal Processing using Sony Spresense](https://github.com/TE-YoshinoriOota/Spresense-Realtime_Signal_Processing-Seminar/blob/main/Documents/Get%20started%20Realtime%20Signal%20Processing%20with%20Sony%20Spresense.pdf)

[Spresenseではじめるリアルタイム信号処理プログラミング](https://github.com/TE-YoshinoriOota/Spresense-Realtime_Signal_Processing-Seminar/blob/main/Documents/Spresense%20%E3%81%A7%E3%81%AF%E3%81%98%E3%82%81%E3%82%8B%E3%83%AA%E3%82%A2%E3%83%AB%E3%82%BF%E3%82%A4%E3%83%A0%E4%BF%A1%E5%8F%B7%E5%87%A6%E7%90%86%E3%83%97%E3%83%AD%E3%82%B0%E3%83%A9%E3%83%9F%E3%83%B3%E3%82%B0.pdf)

## Contents of Sketches
|Sketch|contents|
----|----
|[Spresense_FrontEnd_through](https://github.com/TE-YoshinoriOota/Spresense-Realtime_Signal_Processing-Seminar/blob/main/Sketches/Spresense_FrontEnd_through/Spresense_FrontEnd_through.ino)|Low latency signal processing framework using Spresense Arduino Library|
|[Spresense_FrontEnd_FIR_LPF](https://github.com/TE-YoshinoriOota/Spresense-Realtime_Signal_Processing-Seminar/blob/main/Sketches/Spresense_FrontEnd_FIR_LPF/Spresense_FrontEnd_FIR_LPF.ino)|Realtime Low Pass Filter example using FIR filter|
|[Spresense_FrontEnd_FIR_HPF](https://github.com/TE-YoshinoriOota/Spresense-Realtime_Signal_Processing-Seminar/blob/main/Sketches/Spresense_FrontEnd_Biquad_HPF/Spresense_FrontEnd_Biquad_HPF.ino)|Realtime High Pass Filter example using FIR filter|
|[Spresense_FrontEnd_FIR_BPF](https://github.com/TE-YoshinoriOota/Spresense-Realtime_Signal_Processing-Seminar/blob/main/Sketches/Spresense_FrontEnd_FIR_BPF/Spresense_FrontEnd_FIR_BPF.ino)|Realtime Band Pass Filter example using FIR filter|
|[Spresense_FrontEnd_Biquad_LPF](https://github.com/TE-YoshinoriOota/Spresense-Realtime_Signal_Processing-Seminar/blob/main/Sketches/Spresense_FrontEnd_Biquad_LPF/Spresense_FrontEnd_Biquad_LPF.ino)|Realtime Low Pass Filter example using Biquad IIR filter|
|[Spresense_FrontEnd_Biquad_HPF](https://github.com/TE-YoshinoriOota/Spresense-Realtime_Signal_Processing-Seminar/blob/main/Sketches/Spresense_FrontEnd_Biquad_HPF/Spresense_FrontEnd_Biquad_HPF.ino)|Realtime High Pass Filter example using Biquad IIR filter|
|[Spresense_FrontEnd_Biquad_BPF](https://github.com/TE-YoshinoriOota/Spresense-Realtime_Signal_Processing-Seminar/blob/main/Sketches/Spresense_FrontEnd_Biquad_BPF/Spresense_FrontEnd_Biquad_BPF.ino)|Realtime Band Pass Filter example using Biquad IIR filter|
|[Spresense_FrontEnd_Biquad_Notch](https://github.com/TE-YoshinoriOota/Spresense-Realtime_Signal_Processing-Seminar/blob/main/Sketches/Spresense_FrontEnd_Biquad_Notch/Spresense_FrontEnd_Biquad_Notch.ino)|Realtime Notch Filter example using Biquad IIR filter|
|[Spresense_FrontEnd_STFT](https://github.com/TE-YoshinoriOota/Spresense-Realtime_Signal_Processing-Seminar/blob/main/Sketches/Spresense_FrontEnd_STFT/Spresense_FrontEnd_STFT.ino)|Realtime Short Time Fourie Transform example containing FFT and iFFT|
|[Spresense_FrontEnd_FIR_BPF_VOL](https://github.com/TE-YoshinoriOota/Spresense-Realtime_Signal_Processing-Seminar/blob/main/Sketches/Spresense_FrontEnd_FIR_BPF_VOL/Spresense_FrontEnd_FIR_BPF_VOL.ino)|Variable Band Pass Filter in real-time using two volumes|
|[Spresense_FrontEnd_STFT_voice_changer](https://github.com/TE-YoshinoriOota/Spresense-Realtime_Signal_Processing-Seminar/blob/main/Sketches/Spresense_FrontEnd_STFT_voice_changer/Spresense_FrontEnd_STFT_voice_changer.ino)|Voice Changer in real-time by shifting FFT data using a volume|
|[Spresense_FrontEnd_EchoEffect](https://github.com/TE-YoshinoriOota/Spresense-Realtime_Signal_Processing-Seminar/blob/main/Sketches/Spresense_FrontEnd_EchoEffect/Spresense_FrontEnd_EchoEffect.ino)|Digital Echo Effector using FIR like filter|
|[Spresense_FrontEnd_ReverbEffect](https://github.com/TE-YoshinoriOota/Spresense-Realtime_Signal_Processing-Seminar/blob/main/Sketches/Spresense_FrontEnd_ReverbEffect/Spresense_FrontEnd_ReverbEffect.ino)|Digital Reverb Effector using IIR like filter|
|[Spresense_FrontEnd_STFT_supersonic](https://github.com/TE-YoshinoriOota/Spresense-Realtime_Signal_Processing-Seminar/blob/main/Sketches/Spresense_FrontEnd_STFT_supersonic/Spresense_FrontEnd_STFT_supersonic.ino)|Super Sonic Hearable Filter by shifting FFT data|
|[Spresense_supersonic_communicator](https://github.com/TE-YoshinoriOota/Spresense-Realtime_Signal_Processing-Seminar/tree/main/Sketches/Spresense_supersonic_communicator)|Super Sonic FSK communicator using Short Time Frouie Transform. This sample using multi cores. Main Core is implemented as a Receiver of FSK Super Sonic. Sub Core is implemented as a Transmitter of FSK Super Sonic|
|[Spresense_rfft_wav_recording](https://github.com/TE-YoshinoriOota/Spresense-Realtime_Signal_Processing-Seminar/blob/main/Sketches/Spresense_rfft_wav_recording/Spresense_rfft_wav_recording.ino)|This sample can record 48kHz sounds to an SD card and FFT calculation simultaneously|


## License
- Source codes are under LGPL V2.1
- The textbook is under CC-BY-SA <br>
![image](https://github.com/user-attachments/assets/b4e995f8-34ec-491f-924f-9cb25171d59b)
