#pragma once

#include <SDL3/SDL.h>
//ttf include?

#include "emu.h"

class Emu;

class Ui
{
public:
	Ui();
	~Ui();

	const int SCREEN_WIDTH = 100;
	const int SCREEN_HEIGHT = 100;

	Emu *emu;
	SDL_Window* sdlWindow;
	SDL_Renderer* sdlRenderer;
	SDL_Texture* sdlTexture;
	SDL_Surface* screen;



	void init_window(Emu *e);
	void handle_events();
};

