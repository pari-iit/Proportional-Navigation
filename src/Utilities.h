#ifndef UTILITIES_H
#define UTILITIES_H
#include <vector>
#include <cassert>
/*
Is superseeded by eigen Hence not needed.
*/
template<typename T>
class MatOps{
public:
    
    static std::vector<T> MatrixVectorMultiply(const std::vector<T>& v, const std::vector<std::vector<T> >& m){
        assert (!m.empty());
        assert (v.size() == m[0].size());
        std::vector<T> ret;
        for(size_t i=0;i<m.size(); ++i){
            double val = 0;
            for(size_t j=0;j<m[0].size(); ++j){
                val += m[i][j]*v[j];
            }
            ret.emplace_back(val);
        }
        return ret;
    }

    static std::vector<std::vector<T> > MatrixMatrixMultiply(
        const std::vector<std::vector<T> >& ,m1,
        const std::vectr<std::vector<T> >& ,m2
    ){
        
        assert (!m1.empty() && !m2.empty());
        assert (m1[0].size() == m2.size());
        std::vector<std::vector<T> > retm(m1.size(), std::vector<T>(m2[0].size(), 0));
        
        for(size_t i = 0;i < m1.size(); ++i ){
            for(size_t j=0;j<m2[0].size(); ++j){
                for(size_t k = 0;k < m1[0].size(); ++k ){
                    for(size_t l = 0;l < m2.size(); ++l ){
                        retm[i][j] += m1[i][k]*m2[l][j];
                    }
                }
            }
        }
        return retm;
    }


};



#endif