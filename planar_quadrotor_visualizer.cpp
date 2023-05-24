#include "planar_quadrotor_visualizer.h"

PlanarQuadrotorVisualizer::PlanarQuadrotorVisualizer(PlanarQuadrotor* quadrotor_ptr) : quadrotor_ptr(quadrotor_ptr) {}

void PlanarQuadrotorVisualizer::render(std::shared_ptr<SDL_Renderer>& gRenderer) {
    Eigen::VectorXf state = quadrotor_ptr->GetState();
    int q_x, q_y;
    const int q_width = 202, q_height = 64; 
    float q_theta;
    q_x = state[0];     
    q_y = state[1];
    q_theta = state[2];
    static int dt = 0;
    dt=dt+1;
    if(dt == 5)
    {
      dt = 0;
    }
    
    SDL_SetRenderDrawColor(gRenderer.get(), 0xFF, 0x00, 0x00, 0xFF);
    SDL_Rect eksperyment;
    SDL_Rect podst  = {q_x, q_y, q_width, q_height};   
   
    SDL_Rect animacja = {20+240*dt, 36, 202, 64};
                        
    eksperyment.x = q_x;
    eksperyment.y = q_y;
    eksperyment.w = q_width;
    eksperyment.h = q_height; 
    SDL_Surface* powierzchnia = SDL_LoadBMP("quadrotor.bmp");
    
    SDL_Texture *tekstura = SDL_CreateTextureFromSurface(gRenderer.get(), powierzchnia);
    SDL_RenderCopyEx(gRenderer.get(), tekstura, &animacja, &podst, q_theta, NULL, SDL_FLIP_NONE);
    SDL_FreeSurface(powierzchnia);
    SDL_DestroyTexture(tekstura);
} 
    /*   Eigen::VectorXf state = quadrotor_ptr->GetState();
    int q_x, q_y;
    const int q_width = 200, q_height = 15;
    float q_theta;
    int obrot;//obrot propelerów
    
  static int dt = 0;
  static bool petla = 0;
  if(petla == 0)
    {
      dt=dt+1;
      if(dt == 10)
      {
        petla = 1;
        
      }
    } 
    if(petla == 1)
    {
      dt = dt - 1;
      if(dt == 0)
      {
        petla = 0; 
      }
    }
    obrot = dt;
    q_x = state[0];
    q_y = state[1];
    q_theta = state[2];

    SDL_SetRenderDrawColor(gRenderer.get(), 0xFF, 0x00, 0x00, 0xFF);

    // Draw body
    boxRGBA(gRenderer.get(), q_x, q_y, q_x + q_width, q_y + q_height, 192, 192, 192, 255);

    // Draw arms
    boxRGBA(gRenderer.get(), q_x + 150, q_y - 25, q_x + 158, q_y + 15, 100, 100, 100, 255);
    boxRGBA(gRenderer.get(), q_x + 50, q_y - 25, q_x + 58, q_y + 15, 100, 100, 100, 255);

    // Draw infinity symbol (propellers)
    filledEllipseRGBA(gRenderer.get(), q_x + 40+obrot, q_y - 25, 15-obrot, 6, 0, 0, 255, 255);//l1
    filledEllipseRGBA(gRenderer.get(), q_x + 66-obrot, q_y -25, 15-obrot, 6, 0, 0, 255, 255);//l2
    filledEllipseRGBA(gRenderer.get(), q_x + 140+obrot, q_y - 25, 15-obrot, 6, 0, 0, 255, 255);//p1
    filledEllipseRGBA(gRenderer.get(), q_x + 166-obrot, q_y -25, 15-obrot, 6, 0, 0, 255, 255);//p2

    // Draw center hub for propellers
    filledCircleRGBA(gRenderer.get(), q_x +54, q_y -25, 4, 0, 0, 0, 255);
    filledCircleRGBA(gRenderer.get(), q_x + 154, q_y -25, 4, 0, 0, 0, 255);
}

#include "planar_quadrotor_visualizer.h"// to jest do rysowania tego drona

PlanarQuadrotorVisualizer::PlanarQuadrotorVisualizer(PlanarQuadrotor *quadrotor_ptr): quadrotor_ptr(quadrotor_ptr) {}


void PlanarQuadrotorVisualizer::render(std::shared_ptr<SDL_Renderer> &gRenderer) {
    Eigen::VectorXf state = quadrotor_ptr->GetState();
    int q_x, q_y;
    const int q_width = 200, q_height = 15; 
    float q_theta;

   
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
} */
