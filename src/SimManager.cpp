#include "SimManager.h"
#include <fstream>
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

void SimManager::readFile(){
    std::string line;
    std::ifstream filestream(_ifile);
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
                assert(tokens.size() == 1+ _N_STATES + _N_STATES*_N_STATES);
                double t = std::stof(tokens[0]);
                Eigen::VectorXf x(_N_STATES);
                for(int i=1;i< 1+_N_STATES;++i){
                    x << std::stof( tokens[i] );
                }
                Eigen::MatrixXf P(_N_STATES,_N_STATES);
                for(int i=_N_STATES;i< 1+_N_STATES*_N_STATES;++i){
                    P << std::stof( tokens[i] );
                }
                State s(t,x,P);
                _ics.emplace_back(s);            
            }

            //then get the Q and F of the vehicles
            if(count == 1){
                assert(tokens.size() == _N_STATES*_N_STATES);
            }

        }
    }
}



