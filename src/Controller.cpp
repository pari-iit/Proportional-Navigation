#include "Controller.h"
#include <cassert>
#include <exception> 


Control ConstantControl::generateControlOneStep(const State& st){
        
    try{
        assert(_K.cols() == st.x().size());
    } catch (const std::exception& e){
        printf("Mismatch in Gain and state vector sizes. %s\n", e.what());
        return {0,Eigen::VectorXf::Zero(1,1)};
    }
    return {st.t(),_K*st.x()};
}

void ConstantControl::setGain(const std::any& K){
    try{
        _K = std::any_cast<Eigen::MatrixXf>(K);
    }
    catch(const std::bad_any_cast& e) {
        printf("%s\n",e.what());
    }
}

void ConstantControl::generateControl(const std::vector<State>& st){
    try{
        assert(_K.rows() != 0 && _K.cols() != 0);
    } catch(const std::exception& e){
        printf("Gain is zero. Fix gain. %s\n",e.what());
    }
    
    int nt = st.size();
    for( auto s:st){
        _U_t.emplace_back(generateControlOneStep(s));
    }
}




void ProNav::setGain(const std::any& N){
    try{
        _N = std::any_cast<double>(N);
    }
    catch(const std::bad_any_cast& e) {
        printf("%s\n",e.what());
    }
}


//st is relative state vector. Own state - relative state of the target
void ProNav::generateControl(const std::vector<State>& st){

    for (auto s: st ){
        assert( s.x().size() == _N_STATES*2);// Point mass model
        Eigen::VectorXf x = s.x();
#if _N_STATES == 3
        Eigen::Vector3f r = x.head(_N_STATES), v = x.tail(_N_STATES);
        Eigen::Vector3f omega = ( r.cross(v))/( r.dot(r) );
#elif _N_STATES == 2
        Eigen::Vector2f r = x.head(_N_STATES), v = x.tail(_N_STATES);
        Eigen::Vector2f omega = ( r.cross(v))/( r.dot(r) );
#endif      
        Eigen::VectorXf a  = (r.stableNormalized()).cross(omega);
        a = (-1*_N* (v.norm()) )* a;
        Control u(s.t(),a);
        _U_t.emplace_back(u);
    }
    
}