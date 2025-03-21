#include "ui.h"
#include <SDL3/SDL_events.h>
#include <SDL3/SDL_oldnames.h>
#include <SDL3/SDL_render.h>
#include <SDL3/SDL_video.h>



Ui::Ui()
{
	
}

Ui::~Ui()
{
}

void Ui::init_window(Emu *e)
{
	std::cout << "initializing ui" << std::endl;
	emu = e;
	
	SDL_Init(SDL_INIT_VIDEO);
	SDL_CreateWindowAndRenderer("test", 1024, 764, 0, &sdlWindow, &sdlRenderer);
	
	
//
//	SDL_CreateWindowAndRenderer(SCREEN_WIDTH, SCREEN_HEIGHT, 0, &sdlWindow, &sdlRenderer);
}

void Ui::handle_events()
{
	SDL_Event e;
	while(SDL_PollEvent(&e)){
		if(e.type == SDL_EVENT_QUIT){
			emu->die = true;
		}
	}
}
