#include "Controller.h"
#include <cassert>
#include <exception> 


Control ConstantControl::generateControlOneStep(const State& st){
        
    try{
        assert(_K.cols() == st.x().size());
    } catch (const std::exception& e){
        printf("Mismatch in Gain and state vector sizes. %s\n", e.what());
        return {0,{}};
    }
    return {st.t(),_K*st.x()};
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

