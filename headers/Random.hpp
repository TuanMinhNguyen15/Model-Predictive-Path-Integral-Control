#ifndef _Random_
#define _Random_

#include <random>

// Beta Distribution
class Beta_Distr{
    private:
        float alpha = 10;
        float beta = 10;
        float X,Y;
        std::mt19937 gen;
        std::gamma_distribution<float> gamma_alpha;
        std::gamma_distribution<float> gamma_beta;

        Beta_Distr();
        void I_Update_Param(float alpha, float beta);
        float I_Gen();


    public:
        Beta_Distr(const Beta_Distr&) = delete;

        static Beta_Distr& Get();

        static void Update_Param(float alpha, float beta);

        static float Gen();
        
};


// Uniform Distribution
class Uniform_Distr{
    private:
        std::mt19937 gen;
        float min = -1;
        float max = 1;
        std::uniform_real_distribution<float> uniform_dist;

        Uniform_Distr();
        void I_Update_Param(float min, float max);
        float I_Gen();


    public:
        Uniform_Distr(const Uniform_Distr&) = delete;

        static Uniform_Distr& Get();

        static void Update_Param(float min, float max);

        static float Gen();
        
};

#endif