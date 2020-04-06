#include "SimManager.h"
#include <fstream>
#include <iostream>
#include <cassert>
#include <future>



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

SimManager::SimManager(const std::string& ifile){

    readFile(ifile);
    
    _c = std::make_unique<ProNav>(_N);
    _f = std::make_unique<KalmanFilter>(std::make_shared<LinearDynamics>(_F,_Q), std::make_unique<NonlinearMeasurementModel>());        
    _sim = std::make_shared<Simulator>(_dt,std::make_shared<LinearDynamics>(_F,_Q,_B));
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
                double t = std::stof(tokens[1]);
                Eigen::VectorXf x(NS);
                for(int i=2;i< 2+NS;++i){
                    x(i-2) =  std::stof( tokens[i] );
                }
                
                Eigen::MatrixXf P(NS,NS);
                for(int i=NS+2;i< 2+NS +(NS*NS);++i){
                    int idx =i-(NS+2);
                    int rowv = (int)(idx/NS);
                    int colv = idx - (rowv*NS);
                
                    P(rowv,colv)  = std::stof( tokens[i] );
                }
                State s(t,x,P);
                _ics.emplace_back(s);  

                if (tokens.size() > 2+ NS + NS*NS){
                    int numel = tokens.size() - (2+ NS + NS*NS);
                    assert(numel% (_N_CONTROL + 1)  == 0);
                    int tokcnt = 2+ NS + NS*NS, ccnt = 0;
                    int t = 0; Eigen::VectorXf v(_N_CONTROL);
                    std::vector<Control> uo;
                    for(int i = 0;i< numel; ++i){
                        if (i% (_N_CONTROL + 1) == 0 ){
                            t = std::stof(tokens[tokcnt+i]);
                        }
                        else{
                            v[ccnt++] = std::stof(tokens[tokcnt+i]);
                        }
                        if (ccnt == 3){
                            Control u(t,v);
                            uo.emplace_back(u);
                            ccnt = 0;
                        }

                    }
                    _u.emplace_back(uo);
                }
                else{
                    _u.emplace_back(std::vector<Control>());    /* code */
                }                         
                
            }

            //then get the F, Q and B  of the vehicles. Assume identical robots
            if(count == 1){
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
            //Get the ProNav gain.
            if(count == 2){
                assert(tokens.size() == 1);
                _N = std::stof(tokens[0]);
            }

            //Get the simulation interval.
            if(count == 3){
                assert(tokens.size() == 1);
                _dt = std::stof(tokens[0]);
            }

        }
    }
}

void SimManager::writeFile(const std::string& ofile, const std::vector<State>& st){
    
    std::ofstream filestream(ofile);
    std::unique_lock<std::mutex> lck(_mut);
    if(filestream.is_open()){
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
        filestream.close();
    }
}

void SimManager::generateMeasurement(const std::string measDir){
    std::vector<std::future< std::vector<State> > > futures;
    for (int i = 0;i< _ics.size(); ++i){
        State s = _ics[i];
        double ti = s.t(),tf = _tf[i];
        futures.emplace_back( std::async( std::launch::async,&Simulator::SimulateControl,_sim,std::move(s),std::move(ti), std::move(tf), std::move(_u[i]) ) );        
    }
    std::vector<std::vector<State> > simres;
    std::for_each(futures.begin(), futures.end(), [&simres](std::future<std::vector<State> >& fut){
        simres.emplace_back(fut.get());
    });

    for(int i =0;i<simres.size(); ++i){
        writeFile(measDir + "/" + std::to_string(i) + ".txt",simres.at(i) );        

    }
}



