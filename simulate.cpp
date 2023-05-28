/**
 * SDL window creation adapted from https://github.com/isJuhn/DoublePendulum            
*/
#include "simulate.h"

    Eigen::MatrixXf LQR(PlanarQuadrotor &quadrotor, float dt) { 
    /* Calculate LQR gain matrix */
    Eigen::MatrixXf Eye = Eigen::MatrixXf::Identity(6, 6);
    Eigen::MatrixXf A = Eigen::MatrixXf::Zero(6, 6);
    Eigen::MatrixXf A_discrete = Eigen::MatrixXf::Zero(6, 6);
    Eigen::MatrixXf B(6, 2);
    Eigen::MatrixXf B_discrete(6, 2);
    Eigen::MatrixXf Q = Eigen::MatrixXf::Identity(6, 6);
    Eigen::MatrixXf R = Eigen::MatrixXf::Identity(2, 2);
    Eigen::MatrixXf K = Eigen::MatrixXf::Zero(6, 6);
    Eigen::Vector2f input = quadrotor.GravityCompInput();

    Q.diagonal() << 4e-3, 4e-3, 4e3, 8e-2, 4.5e-4, 2 / 2 / M_PI;
    R.row(0) << 3e1, 7;
    R.row(1) << 7, 3e1;                                        

    std::tie(A, B) = quadrotor.Linearize();
    A_discrete = Eye + dt * A;
    B_discrete = dt * B;
    
    return LQR(A_discrete, B_discrete, Q, R);
}

void control(PlanarQuadrotor &quadrotor, const Eigen::MatrixXf &K) {
    Eigen::Vector2f input = quadrotor.GravityCompInput();
    quadrotor.SetInput(input - K * quadrotor.GetControlState());
}

void wykres_x (std::vector<float> x, std::vector<float> y, std::vector<float> theta, std::vector<float> t)
{
    using namespace matplot;
    plot(t, x, "-x");
    hold(on);
    plot(t, y, "-y");
    plot(t, theta, "-o");
}

int main(int argc, char* args[])
{
    std::shared_ptr<SDL_Window> gWindow = nullptr;
    std::shared_ptr<SDL_Renderer> gRenderer = nullptr;
    const int SCREEN_WIDTH = 1280;
    const int SCREEN_HEIGHT = 720;
    const float SCREEN_W_CENTER = SCREEN_WIDTH/2;
    const float SCREEN_H_CENTER = SCREEN_HEIGHT/2;
    float volume = 128;
    /**
     * TODO: Extend simulation
     * 1. Set goal state of the mouse when clicking left mouse button (transform the coordinates to the quadrotor world! see visualizer TODO list)
     *    [x, y, 0, 0, 0, 0]
     * 2. Update PlanarQuadrotor from simulation when goal is changed
    */
    Eigen::VectorXf initial_state = Eigen::VectorXf::Zero(6); 
    initial_state(0) = 640;
    initial_state(1) = 360;
    
    PlanarQuadrotor quadrotor(initial_state);
    PlanarQuadrotorVisualizer quadrotor_visualizer(&quadrotor);
    quadrotor.playMusic("dron_fly.wav", volume);
    /**
     * Goal pose for the quadrotor
     * [x, y, theta, x_dot, y_dot, theta_dot]  
     * For implemented LQR controller, it has to be [x, y, 0, 0, 0, 0]                  
    */
    Eigen::VectorXf goal_state = Eigen::VectorXf::Zero(6);
    goal_state << 200, 200, 0, 0, 0, 0;             
    quadrotor.SetGoal(goal_state);                 
    /* Timestep for the simulation */
    const float dt = 0.1;
    Eigen::MatrixXf K = LQR(quadrotor, dt);
    Eigen::Vector2f input = Eigen::Vector2f::Zero(2);           

    /**
     * TODO: Plot x, y, theta over time
     * 1. Update x, y, theta history vectors to store trajectory of the quadrotor
     * 2. Plot trajectory using matplot++ when key 'p' is clicked            // zrobione (na 99% nic się nie zepsuje)     
    */
    std::vector<float> x_history;
    std::vector<float> y_history;
    std::vector<float> theta_history;
    std::vector<float> time_history;
    float time_current = dt;

    std::vector<float>::iterator iterX = x_history.begin();
    std::vector<float>::iterator iterY = y_history.begin();

    if (init(gWindow, gRenderer, SCREEN_WIDTH, SCREEN_HEIGHT) >= 0)
    {
        SDL_Event e;
        bool quit = false;
        float delay;
        int x, y;               // lokalny x y 
        int ts;
        Eigen::VectorXf state = Eigen::VectorXf::Zero(6);

        while (!quit)
        {  
            //events
            while (SDL_PollEvent(&e) != 0)      
            {                                                 
                if (e.type == SDL_QUIT)
                {
                    quit = true;
                }
                else if (e.type == SDL_MOUSEMOTION)
                {
                    SDL_GetMouseState(&x, &y);
                    std::cout << "Mouse position: (" << x << ", " << y << ")" << std::endl;
                }
                else if (e.type == SDL_MOUSEBUTTONDOWN)
                {
                    SDL_GetMouseState(&x, &y);
                    std::cout << "Mouse WHEN CLICKED : (" << x << ", " << y << ")" << std::endl;
                    goal_state(0) = x;
                    goal_state(1) = y;  
                    quadrotor.SetGoal(goal_state);
                }
                else if (e.type == SDL_KEYDOWN)
                {
                    if(e.key.keysym.sym == SDLK_p)
                    {
                        std::cout << "Pressed p" << std::endl;
                        wykres_x(x_history, y_history, theta_history ,time_history);
                    }
                }
            }
            Eigen::VectorXf history = quadrotor.GetState();         // zapisywanie w czasie
            x_history.push_back(history(0));
            y_history.push_back(history(1));
            theta_history.push_back(history(2));
            time_history.push_back(time_current);
            time_current = time_current + dt;

            iterX=x_history.end();
            iterY=y_history.end();

            quadrotor.getDistance(x_history.end()[-2], y_history.end()[-2], x_history.end()[-1], y_history.end()[-1], SCREEN_W_CENTER, SCREEN_H_CENTER, volume);

            SDL_Delay((int) dt * 1000);
            
            SDL_SetRenderDrawColor(gRenderer.get(), 0xFF, 0xFF, 0xFF, 0xFF);
            SDL_RenderClear(gRenderer.get());

            /* Quadrotor rendering step */
            quadrotor_visualizer.render(gRenderer);

            SDL_RenderPresent(gRenderer.get());

            /* Simulate quadrotor forward in time */
            control(quadrotor, K);
            quadrotor.Update(dt);
            }
    }
    SDL_Quit();
    quadrotor.cleanMusic();
    return 0;
}

int init(std::shared_ptr<SDL_Window>& gWindow, std::shared_ptr<SDL_Renderer>& gRenderer, const int SCREEN_WIDTH, const int SCREEN_HEIGHT)
{
    if (SDL_Init(SDL_INIT_VIDEO) >= 0)
    {
        SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY, "1");
        gWindow = std::shared_ptr<SDL_Window>(SDL_CreateWindow("Planar Quadrotor", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_SHOWN), SDL_DestroyWindow);
        gRenderer = std::shared_ptr<SDL_Renderer>(SDL_CreateRenderer(gWindow.get(), -1, SDL_RENDERER_ACCELERATED), SDL_DestroyRenderer);
        SDL_SetRenderDrawColor(gRenderer.get(), 0xFF, 0xFF, 0xFF, 0xFF);
    }
    else
    {
        std::cout << "SDL_ERROR: " << SDL_GetError() << std::endl;
        return -1;
    }
    return 0;
}
