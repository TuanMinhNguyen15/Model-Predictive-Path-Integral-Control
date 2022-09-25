#include "Random.hpp"

// Beta Distribution
Beta_Distr::Beta_Distr(){
    this->gen.seed((std::random_device())()); 
    this->gamma_alpha.param(std::gamma_distribution<float>::param_type(this->alpha,1));
    this->gamma_beta.param(std::gamma_distribution<float>::param_type(this->beta,1));
}

float Beta_Distr::I_Gen(){
    this->X = this->gamma_alpha(this->gen);
    this->Y = this->gamma_beta(this->gen);

    return X/(X+Y);
}

void Beta_Distr::I_Update_Param(float alpha, float beta){
    this->alpha = alpha;
    this->beta = beta;
    this->gamma_alpha.param(std::gamma_distribution<float>::param_type(this->alpha,1));
    this->gamma_beta.param(std::gamma_distribution<float>::param_type(this->beta,1));
}

Beta_Distr& Beta_Distr::Get(){
    static Beta_Distr Instance;
    return Instance;
}

void Beta_Distr::Update_Param(float alpha, float beta){
    return Get().I_Update_Param(alpha,beta);
}

float Beta_Distr::Gen(){
    return Get().I_Gen();
}


// Uniform Distribution
Uniform_Distr::Uniform_Distr(){
    this->gen.seed((std::random_device())()); 
    this->uniform_dist.param(std::uniform_real_distribution<float>::param_type(this->min,this->max));
    
}

float Uniform_Distr::I_Gen(){
    return this->uniform_dist(this->gen);
}

void Uniform_Distr::I_Update_Param(float min, float max){
    this->min = min;
    this->max = max;
    this->uniform_dist.param(std::uniform_real_distribution<float>::param_type(this->min,this->max));
}

Uniform_Distr& Uniform_Distr::Get(){
    static Uniform_Distr Instance;
    return Instance;
}

void Uniform_Distr::Update_Param(float min, float max){
    return Get().I_Update_Param(min,max);
}

float Uniform_Distr::Gen(){
    return Get().I_Gen();
}
