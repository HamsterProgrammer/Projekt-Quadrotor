#ifndef AUDIO_H
#define AUDIO_H
#include <SDL_mixer.h>
#include <SDL_main.h>

class AudioManager {
public:
    AudioManager();
    ~AudioManager();

    void playMusic(const char* filePath, int volume);
    void setVolume(int volume);

private:
    Mix_Music* music;
};

#endif 