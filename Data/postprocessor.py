import matplotlib.pyplot as plt
import numpy as np
import os,glob
class Traj:
    def __init__(self,t,x,P):
        self._t = t
        self._x = x
        self._P = P


    
def plotTargetPaths(dirname = '../Data/',ns = 6):
    Trajs = []
    for file in glob.glob(dirname+'./[0-9].txt'):
        fid  = open(os.path.join(dirname,file),"r")
        A = fid.readlines()
        fid.close()
        
        T = []; X = [];P = []
        for i in range(len(A)):
            A[i] = A[i].split(',');        
            try:
                assert(len(A[i]) == 1+ns + ns*ns)
            except:
                print(len(A[i]), 1+ns + ns*ns)
            t = np.double(A[i][0])            
            x = []
            for j in range(ns):
                x.append(np.double(A[i][j+1]) )
            p = []
            for j in range(ns*ns):
                p.append(np.double(A[i][j+1+ns]))
            x = np.array(x); p = np.array(p)
            p = np.reshape(p, (ns,ns))
            
            T.append(t); 
            X.append(x)
            P.append(p)
        T = np.array(T); X = np.array(X)
        traj = Traj(T,X,P)
        Trajs.append(traj)
    

    i = 1
    for traj in Trajs:
        print(np.shape(traj._x))
        plt.plot(traj._x[0,0],traj._x[0,1],'o')
        plt.plot(traj._x[:,0],traj._x[:,1],'--x', label = str(i))
        i = i+1
    plt.legend()
    plt.title('x-y trace')
    plt.show()

def readInputFile(dirname = '../Data/',ns = 6):

if __name__ == '__main__':
    plotTargetPaths()
    
    



