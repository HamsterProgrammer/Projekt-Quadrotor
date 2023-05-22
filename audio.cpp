#include <SDL.h>
#include "sdltemplate.h"
#include <SDL_mixer/SDL_mixer.h>
#include <vector>
#include <iostream>

int loadMusic(const char* filename){}
int loadSound(const char* filename){}

int volume;
void setVolume(int v){}
int playSound(int m){}

int initMixer(){}
int quitMixer(){}

int main(){
    sdltemplate::init();
    initMixer();
    int sound = loadSound("letayuschiy-dron.wav");

    return 0;
} 