#include "Controller.h"
#include <cassert>
#include <exception> 


Control ConstantControl::generateControlOneStep(const State& st){
        
    try{
        assert(_K[0].size() == st.x().size());
    } catch (const std::exception& e){
        printf("Mismatch in Gain and state vector sizes. %s\n", e.what());
        return {0,{}};
    }
    std::vector<double> u(_K.size(),0.0);
    for(int i=0;i< u.size();++i){
        for(int j=0;j<st.x().size(); ++j){
            u[i]+=_K[i][j]*st.x()[j];//u = Kx
        }
    }

    return {st.t(),u};
}

void ConstantControl::generateControl(const std::vector<State>& st){
    try{
        assert(!_K.empty());
    } catch(const std::exception& e){
        printf("Gain is zero. Fix gain. %s\n",e.what());
    }
    
    int nt = st.size();
    for( auto s:st){
        _U_t.emplace_back(generateControlOneStep(s));
    }
}

