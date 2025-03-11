#pragma once

#include <SDL3/SDL.h>
#include "emu.h"

class Emu;

class ui
{
public:
	ui(Emu* e);
	~ui();

	const int SCREEN_WIDTH = 1024;
	const int SCREEN_HEIGHT = 768;

	Emu *emu;
	SDL_Window* sdlWindow;
	SDL_Renderer* sdlRenderer;
	SDL_Texture* sdlTexture;
	SDL_Surface* screen;



	void init_window();
	void handle_events();
};

