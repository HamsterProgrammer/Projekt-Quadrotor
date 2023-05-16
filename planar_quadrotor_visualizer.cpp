#include "planar_quadrotor_visualizer.h"// to jest do rysowania tego drona

PlanarQuadrotorVisualizer::PlanarQuadrotorVisualizer(PlanarQuadrotor *quadrotor_ptr): quadrotor_ptr(quadrotor_ptr) {}

/*
 * TODO: Improve visualizetion
 * 1. Transform coordinates from quadrotor frame to image frame so the circle is in the middle of the screen
 * 2. Use more shapes to represent quadrotor (e.x. try replicate http://underactuated.mit.edu/acrobot.html#section3 or do something prettier)
 * 3. Animate proppelers (extra points)
 */
void PlanarQuadrotorVisualizer::render(std::shared_ptr<SDL_Renderer> &gRenderer) {
    Eigen::VectorXf state = quadrotor_ptr->GetState();
    int q_x, q_y;
    const int q_width = 200, q_height = 15; 
    float q_theta;

    /* x, y, theta coordinates -- kordy kółka i wychylenia */
    q_x = state[0];     
    q_y = state[1];
    q_theta = state[2];
    
    SDL_SetRenderDrawColor(gRenderer.get(), 0xFF, 0x00, 0x00, 0xFF);
    SDL_Rect eksperyment;
    SDL_Rect podst  = {q_x, q_y, q_width, q_height};   // to jest okienko na którym to się buduje zrb że okienko porusza się
    SDL_Rect lacz1  = {q_x+150, q_y-25, 10, 40};
    SDL_Rect lacz2  = {q_x+50, q_y-25, 10, 40};                              
    eksperyment.x = q_x;
    eksperyment.y = q_y;
    eksperyment.w = q_width;
    eksperyment.h = q_height; 

    SDL_Surface* powierzchnia = SDL_CreateRGBSurface(0, 1680, 720, 32, 0, 0, 0, 0);
    SDL_FillRect(powierzchnia, nullptr, 0xFF0000FF);
    SDL_Texture *tekstura = SDL_CreateTextureFromSurface(gRenderer.get(), powierzchnia);
    

    SDL_RenderCopyEx(gRenderer.get(), tekstura, NULL , &podst, q_theta, NULL, SDL_FLIP_NONE);
    SDL_RenderCopyEx(gRenderer.get(), tekstura, NULL, &lacz1, q_theta, NULL, SDL_FLIP_NONE);
    SDL_RenderCopyEx(gRenderer.get(), tekstura, NULL, &lacz2, q_theta, NULL, SDL_FLIP_NONE);//trzeba je jakos polaczyc

    SDL_FreeSurface(powierzchnia);
  
  
    SDL_DestroyTexture(tekstura);
   //  SDL_RenderFillRect(gRenderer.get(), &eksperyment);
   // filledCircleColor(gRenderer.get(), q_x, q_y, 30, 0xFF0000FF);

   
   

}