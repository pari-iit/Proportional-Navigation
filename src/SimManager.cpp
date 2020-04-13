#include "SimManager.h"
#include <fstream>
#include <iostream>
#include <cassert>
#include <future>
#include <unordered_set>
#include <random>
#include <cmath>
#define _USE_KF 1

static std::vector<std::string> tokenizer(const std::string& s, const char& delimiter = ' '){
  std::istringstream str_tok(s);
  std::vector<std::string> tokens;
  
  std::string intermediate;
  while(getline(str_tok,intermediate,delimiter)){
    intermediate.erase(remove(intermediate.begin(), intermediate.end(), ' '), intermediate.end()); 
    if (intermediate.length())
      tokens.push_back(intermediate);
  }
  
  return tokens;

}

template<typename T>
std::vector<T> slice(std::vector<T> const& v, int m, int n)
{
    auto first = v.cbegin() + m;
    auto last = v.cbegin() + n ;

    std::vector<T> vec(first, last);
    return vec;
}

SimManager::SimManager(const std::string& ifile){

    readFile(ifile);
    
    _c = std::make_unique<ProNav>(_N);
    _m = std::make_unique<NonlinearMeasurementModel>();
    _f = std::make_unique<KalmanFilter>(std::make_shared<LinearDynamics>(_F,_Q), std::make_unique<NonlinearMeasurementModel>());        
    _sim = std::make_shared<Simulator>(_dt,std::make_shared<LinearDynamics>(_F,_Q,_B));
}

State SimManager::readState(const std::vector<std::string>& tokens){

    double t = std::stof(tokens[0]);
    Eigen::VectorXf x(NS);
    for(int i=1;i< 1+NS;++i){
        x(i-1) =  std::stof( tokens[i] );
    }
                
    Eigen::MatrixXf P(NS,NS);
    for(int i=NS+1;i< 1+NS +(NS*NS);++i){
        int idx =i-(NS+1);
        int rowv = (int)(idx/NS);
        int colv = idx - (rowv*NS);
                
        P(rowv,colv)  = std::stof( tokens[i] );
    }
    State s(t,x,P);
    return s;

}

std::vector<Control> SimManager::readControlSequence(const std::vector<std::string>& tokens){

    int numel = tokens.size();
    int ccnt = 0;
    int t = 0; Eigen::VectorXf v(_N_CONTROL);
    std::vector<Control> uo;
    
    for(int i = 0;i< numel; ++i){
        
        if (i% (_N_CONTROL + 1) == 0 ){
            t = std::stof(tokens[i]);
        }
        else{

            v[ccnt++] = std::stof(tokens[i]);
        }
        if (ccnt == 3){
            Control u(t,v);
            uo.emplace_back(u);
            ccnt = 0;
        }

    }
    return uo;    
}

void SimManager::readFile(const std::string& ifile){
    std::string line;
    std::ifstream filestream(ifile);
    if (filestream.is_open()) {
        int count = 0;
        while (std::getline(filestream, line)) {
            //Comma separated string
            
            std::vector<std::string> tokens = tokenizer(line,',');

            if (tokens.size() == 0) {
                count++; 
                continue;
            }
            //first get the target vehicle initial conditions
            if (count == 0){
                assert(tokens.size() >= 2+ NS + NS*NS);                
                _tf.emplace_back(std::stof(tokens[0]));
                
                State s = readState(slice(tokens,1,2+ NS + NS*NS));
                
                _ics.emplace_back(s);  

                if (tokens.size() > 2+ NS + NS*NS){
                    int numel = tokens.size() - (2+ NS + NS*NS);
                    assert(numel% (_N_CONTROL + 1)  == 0);
                    int tokcnt = 2+ NS + NS*NS;                    
                    std::vector<Control> uo = readControlSequence(slice(tokens,tokcnt,tokens.size()) );                    
                    _u.emplace_back(uo);
                }
                else{
                    _u.emplace_back(std::vector<Control>());    /* code */
                }                         
                
            }
            //Get the initial states for the interceptors.
            if (count == 1){
                assert(tokens.size() >= 1+ NS + NS*NS);                
                State s = readState(slice(tokens,0,1+ NS + NS*NS));                
                _iloc.emplace_back(s);  

                if (tokens.size() > 1+ NS + NS*NS){
                    int numel = tokens.size() - (1+ NS + NS*NS);
                    assert(numel == (_N_CONTROL + 1) );
                    int tokcnt = 1+ NS + NS*NS;                    
                    std::vector<Control> uo = readControlSequence(slice(tokens,tokcnt,tokens.size()) );                 
                    assert (uo.size() == 1);   
                    assert (s.t() == uo[0].time());
                    _iu.emplace_back(uo[0]);
                }
                else{
                    _iu.emplace_back( Control(s.t(),Eigen::VectorXf::Zero(_N_CONTROL)) );    /* code */
                }                         
                
            }

            //then get the F, Q and B  of the vehicles. Assume identical robots
            if(count == 2){
                assert(tokens.size() >= 2*NS*NS);
                _F = Eigen::MatrixXf::Identity(NS,NS);
                _Q = Eigen::MatrixXf::Identity(NS,NS);
                for(int i=0;i<NS*NS;++i){
                    int rnum = i/NS;
                    int cnum = i - (rnum*NS);
                    _F(rnum,cnum) = std::stof(tokens[i]);
                }

                for(int i=0;i<NS*NS;++i){
                    int rnum = i/NS;
                    int cnum = i - (rnum*NS);
                    _Q(rnum,cnum) = std::stof(tokens[i+NS*NS]);
                }
                if (tokens.size() > NS*NS*2){

                    int numel = tokens.size()-NS*NS*2;
                    assert (numel = _N_CONTROL*NS);
                    // int ncols = numel/NS;
                    //Proper number of elements required to complete matrix.
                    // assert(ncols*NS == numel); 
                    _B = Eigen::MatrixXf::Zero(NS,_N_CONTROL);
                    for(int i=0;i<numel;++i){
                        int rnum = i/(_N_CONTROL);
                        int cnum = i - (rnum*_N_CONTROL);
                        _B(rnum,cnum) = std::stof(tokens[i+NS*NS*2]);
                    }
                }
                else{
                    _B = Eigen::MatrixXf::Zero(NS,1);
                }

            }

            //Get the R Matrix
            if(count == 3){
                assert(tokens.size() == _NM*_NM);
                _R = Eigen::MatrixXf::Zero(_NM,_NM);
                for(int i=0;i < tokens.size(); ++i){
                    int rnum = i/_NM;
                    int cnum = i-(rnum*_NM);
                    _R(rnum,cnum) = std::stof(tokens[i]);
                }
            }

            //Get the ProNav gain.
            if(count == 4){
                assert(tokens.size() == 1);
                _N = std::stof(tokens[0]);
            }

            //Get the simulation interval.
            if(count == 5){
                assert(tokens.size() == 1);
                _dt = std::stof(tokens[0]);
            }

            //Get tolerance criterion

            if(count == 6){
                assert(tokens.size() == 1);
                _tol = std::stof(tokens[0]);
            }

        }
    }
}

void SimManager::writeStates(std::ofstream& filestream, const std::vector<State>& st){
    for(int i=0;i<st.size();++i){
        Eigen::VectorXf x = st.at(i).x();
        Eigen::MatrixXf P = st.at(i).P();
        filestream << st.at(i).t();
        filestream << ", ";
        for(int j=0;j< x.size(); ++j){
            filestream << x(j);
            filestream << ",";
        }
        filestream <<" ";
        for(int j=0;j< P.rows(); ++j){
            for(int k=0;k < P.cols();++k){
                filestream << P(j,k);
                if (j < P.rows()-1 || k <P.cols()-1){
                    filestream << ",";
                }
                    
            }
        }
        filestream <<"\n";
    }
}


void SimManager::writeFile(const std::string& ofile, const Results& res){
    
    std::vector<State> ist = res.i_seq;
    std::vector<State> tst = res.t_seq;

    std::ofstream filestream(ofile);
    std::unique_lock<std::mutex> lck(_mut);
    if(filestream.is_open()){
        writeStates(filestream,tst);
        filestream << "\n\n";
        writeStates(filestream,ist);
        filestream.close();
    }
}

 std::unordered_map<int,int> SimManager::mapTargetToInterceptor(){
    std::unordered_map<int,int> t_to_i;

    assert(_ics.size() == _iloc.size());
    std::unordered_set<int> assigned;
     for (int i=0;i<_ics.size(); ++i){
         int minidx = 0;         
         double mindist = __DBL_MAX__;
         for(int j=0;j<_iloc.size(); ++j){
             if (assigned.find(j) != assigned.end() ) continue;
             int d = _ics[i].getDistance(_iloc[j]);
             if( mindist > d){
                 mindist = d;
                 minidx = j;
             }            
         }         
         t_to_i[i] = minidx;
         assigned.insert(minidx);
     }
    return t_to_i;
 }

Results SimManager::simulateProNav(const int&& t_id, const int&& i_id){
    //sim target one step t_k->t_k+1;
    //generate measurement  at t_k+1
    //add noise to measurement at t_k+1;
    //get estimated position by using Kalman filter at t_k+1;
    // use estimated position to generate relative measurement t_k-t_k+1;
    // use relative measurment to get control at t_k
    // move interceptor t->t_k+1;
    //go to step 1;
    //repeat until distance minimized.     
    
    double dist = __DBL_MAX__;
    State t_st = std::move(_ics[t_id]);//actual target state
    std::vector<Control> ut = std::move(_u[t_id]); //actual target control
    std::vector<State> t_st_seq;t_st_seq.push_back(t_st);

#if _USE_KF
    //Move the initial point randomly
    std::unique_lock<std::mutex> ulock(_mut);
    std::random_device rd;
    std::mt19937 eng(rd());    
    Eigen::VectorXf ex =t_st.x();
    Eigen::MatrixXf eP =t_st.P();
    for (int i=0; i < ex.size(); ++i){
        std::normal_distribution<double> pdf(0,0.01*eP(i,i) );
        ex(i) = ex(i) + pdf(eng);
    }        
    ulock.unlock();
    State et_st(t_st.t(),ex,eP); // estimate carried on by the onboard seeker.
    //interceptor state and controls
#endif

    State i_st = std::move(_iloc[i_id]);    
    Control ui = std::move(_iu[i_id]);    
    std::vector<State> i_st_seq;
    i_st_seq.push_back(i_st);
    
    std::vector<double> disthist;
    while(dist > _tol && t_st.t() <_tf[t_id]){
        //Actual target simulation        
        _sim->SimStep(t_st,ut);
        t_st_seq.push_back(t_st);

        //Generation of radar measurment;
        State relst(t_st.t(),t_st.x()-i_st.x(),t_st.P()+i_st.P());        


#if (_USE_KF)        
        ulock.lock();
        Measurement m = _m->generateNoisyMeasurement(std::move(relst),_R);
        ulock.unlock();

        //EVERYTHING HERE ON AFTER IS WHAT GOES ON IN THE ONBOARD SEEKER.
        //Kalman filter to estimate the target. Should be tuned separately different targets.
        //set _USE_KF to zero if you want to use direct measurements.         
        //Estimate postion of the target
        _f->update(et_st,m,i_st,_dt);
        // get relative state of the target with respect to the interceptor        
        relst = State(et_st.t(),et_st.x()-i_st.x(),et_st.P()+i_st.P());
#endif        
        //generate control
        _c->generateControl({relst});        

      
        //Apply control to the interceptor to move it
        _sim->SimStep(i_st,_c->getControl());
        
        //Recording and termination criteria checking 
        //Save state sequence
        i_st_seq.push_back(i_st);
        dist = i_st.getDistance(t_st);


        if (disthist.size() < SimManager::_dist_buffer_check){
            disthist.push_back(dist);
        }
        else{
            
            if (std::all_of(disthist.begin(),disthist.end(),[dist](const double& d){
                return (d <= dist);
            })){
                // ulock.unlock();
                break;
            }
            disthist.erase(disthist.begin());
            disthist.push_back(dist);
        }

        if ( fabs(std::nearbyint(t_st.t()) - t_st.t()) < 1.0e-5 ){
            std::cout << "Thread id = " << (std::this_thread::get_id()) 
                      << " time = " << t_st.t() << " dist = " << dist << std::endl;            
        }
    }
    std::cout << "Thread id = " << (std::this_thread::get_id())
              << ". Time = " << i_st.t()
              << ". Dist = " << dist << std::endl;                
    std::this_thread::sleep_for(std::chrono::milliseconds(1));


    return {t_st_seq,i_st_seq};
     
}

void SimManager::runSimulation(const std::string& resDir){
    std::unordered_map<int,int> t_to_i = mapTargetToInterceptor();
    std::vector<std::future< Results > > futures;
    for (int i = 0;i< _ics.size(); ++i){
        int targ= i; int icp = t_to_i[i];        
        
        futures.emplace_back( std::async( std::launch::async,&SimManager::simulateProNav,this,std::move(targ),std::move(icp) ) );        
    }
    std::vector<Results> simres;
    std::for_each(futures.begin(), futures.end(), [&simres](std::future<Results >& fut){
        simres.emplace_back(fut.get());
    });

    for(int i =0;i<simres.size(); ++i){
        writeFile(resDir + "/res_" + std::to_string(i) + ".txt",simres.at(i) );        

    }
}