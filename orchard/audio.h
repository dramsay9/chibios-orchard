#ifndef __ORCHARD_AUDIO_H__
#define __ORCHARD_AUDIO_H__

struct audio_data {
  int x;
  int y;
  int z;
};

void audioStart(I2CDriver *driver);
void audioStop(void);
msg_t audioPoll(struct audio_data *data);

#endif /* __ORCHARD_AUDIO_H__ */
