#include "SimManager.h"
#include <fstream>
#include <iostream>
#include <cassert>



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
    _sim = std::make_unique<Simulator>(_dt,std::make_shared<LinearDynamics>(_F,_Q,_B));
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
                
                assert(tokens.size() == 1+ NS + NS*NS);                
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
                _ics.emplace_back(s);            
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
                    assert (numel > NS);
                    int ncols = numel/NS;
                    //Proper number of elements required to complete matrix.
                    assert(ncols*NS == numel); 
                    _B = Eigen::MatrixXf::Zero(NS,ncols);
                    for(int i=0;i<numel;++i){
                        int rnum = i/ncols;
                        int cnum = i - (rnum*ncols);
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



