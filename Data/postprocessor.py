#!/usr/bin/python3
import numpy as np
import matplotlib.pyplot as plt
import os,glob
import matplotlib.animation as animation
from mpl_toolkits import mplot3d
import mpl_toolkits.mplot3d.axes3d as p3
import sys 

def printUsage():
    print('Usage: python3 postprocessor.py <dof>')

class Traj:
    def __init__(self,t,x,P):
        self._t = t
        self._x = x
        self._P = P


def readInputFile(dirname = '../Data/',ns = 6):
    Trajs = []
    
    for file in glob.glob(dirname+'./res_[0-9].txt'):
        fid  = open(os.path.join(dirname,file),"r")
        A = fid.readlines()
        fid.close()

        startpt = 0
        tT = []; tX = [];tP = []        
        for i in range(len(A)):
            A[i] = A[i].split(',')
            if len(A[i]) < 1+ ns +ns*ns:
                startpt = i
                break
            t = np.double(A[i][0])            
            x = []
            for j in range(ns):
                x.append(np.double(A[i][j+1]) )
            p = []
            for j in range(ns*ns):
                p.append(np.double(A[i][j+1+ns]))
            x = np.array(x); p = np.array(p)
            p = np.reshape(p, (ns,ns))
            
            tT.append(t)
            tX.append(x)
            tP.append(p)
        tT = np.array(tT); tX = np.array(tX)
        ttraj = Traj(tT,tX,tP)
        
        iT = []; iX = [];iP = []
        for i in range(startpt+1,len(A)):            
            A[i] = A[i].split(',')     
            if len(A[i]) < 1+ ns +ns*ns:
                continue
            t = np.double(A[i][0])            
            x = []
            for j in range(ns):
                x.append(np.double(A[i][j+1]) )
            p = []
            for j in range(ns*ns):
                p.append(np.double(A[i][j+1+ns]))
            x = np.array(x); p = np.array(p)
            p = np.reshape(p, (ns,ns))
            
            iT.append(t)
            iX.append(x)
            iP.append(p)
        iT = np.array(iT); iX = np.array(iX)
        itraj = Traj(iT,iX,iP)
        Trajs.append([ttraj,itraj])
    return Trajs


    
def create_movie_3d(Trajs, ns = 6,saveVideo = False):    
    def processXdata(Traj):                                    
        onlyx = [x for x in Traj._x]
        onlyx = np.squeeze(onlyx)
        return np.squeeze(np.array(onlyx))
        
    x_list = [np.array(processXdata(t)) for traj in Trajs for t in traj ]

    for x in x_list:
        assert np.shape(x)[0] > 2
        assert np.shape(x)[1] == ns
    
    
    fig = plt.figure()
    ax = plt.axes(projection='3d')

    cnt = 0
    color = ['r','b','g','k']
    marker = ['o','^','D','+']
    for data in x_list:        
        ax.plot3D([data[0,0]],[data[0,1]],[data[0,2]], marker = marker[(int)(cnt%2)])
        ax.plot3D(data[:,0],data[:,1],data[:,2],c = color[(int)(cnt/2)])
    
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')

        ax.set_title( 'Target = '+ marker[0]+' Interceptor = '+ marker[1] )
        cnt = cnt + 1
        if cnt == 8: 
            cnt = 0

    

    
    fig = plt.figure()
    ax = p3.Axes3D(fig)   
    lines = [ax.plot(dat[0:1,0], dat[0:1,1], dat[0:1,2])[0] for dat in x_list]
    points = [ax.plot([dat[0,0]], [dat[0,1]], [dat[0,2]],'x')[0] for dat in x_list]
    
    max_len = 0
    for x in x_list:
        if np.shape(x)[0] > max_len:
            max_len = np.shape(x)[0]

    max_lim = np.zeros((ns,1), dtype = np.double)
    min_lim = np.full((ns,1), np.inf, dtype = np.double)

    for k in range(ns):   
        for x in x_list:     
            max_lim[k] = max(max_lim[k], max(x[:,k]))
            min_lim[k] = min(min_lim[k], min(x[:,k]))
    
    def update_lines(num, dataLines, lines, points) :
        for line, data, pt in zip(lines, dataLines,points) :
            # NOTE: there is no .set_data() for 3 dim data...                                    
            r,_ = np.shape(data)
        
            val = num
            if num > r:
                val = r
            line.set_data( np.transpose(data[:val,0:2]) )            
            line.set_3d_properties(np.transpose(data[:val,2]))
            pt.set_data( np.transpose(data[val,0:2]))
            pt.set_3d_properties(np.transpose(data[val,2]))
        return lines
    

    ax.set_xlim3d([min_lim[0], max_lim[0]])
    ax.set_xlabel('X')

    ax.set_ylim3d([min_lim[1], max_lim[1]])
    ax.set_ylabel('Y')

    ax.set_zlim3d([min_lim[2], max_lim[2]])
    ax.set_zlabel('Z')
    
    
    ax.set_title('Result Animation')

    # Creating the Animation object
    line_ani = animation.FuncAnimation(fig, update_lines, frames = max_len, fargs=(x_list, lines,points),
                                   interval=1, blit=False)

    if saveVideo:
        from time import gmtime, strftime
        time_str = strftime("%Y-%m-%d-%H-%M-%S", gmtime())
        line_ani.save('Traj_video_'+time_str+'_.mp4', fps=25, \
                        extra_args=['-vcodec', 'libx264'])

    plt.show()

def create_movie_2d(Trajs, ns = 4,saveVideo = False):    
    def processXdata(Traj):                                    
        onlyx = [x for x in Traj._x]
        onlyx = np.squeeze(onlyx)
        return np.squeeze(np.array(onlyx))
        
    x_list = [np.array(processXdata(t)) for traj in Trajs for t in traj ]

    for x in x_list:
        assert np.shape(x)[0] > 2
        assert np.shape(x)[1] == ns
    
    
    fig = plt.figure()
    ax = plt.axes()

    cnt = 0
    color = ['r','b','g','k']
    marker = ['o','^','D','+']
    for data in x_list:        
        ax.plot([data[0,0]],[data[0,1]], marker = marker[(int)(cnt%2)])
        ax.plot(data[:,0],data[:,1],c = color[(int)(cnt/2)])
    
        ax.set_xlabel('X')
        ax.set_ylabel('Y')        

        ax.set_title( 'Target = '+ marker[0]+' Interceptor = '+ marker[1] )
        cnt = cnt + 1
        if cnt == 8: 
            cnt = 0

    

    
    fig = plt.figure()
    ax = plt.axes()  
    lines = [ax.plot(dat[0:1,0], dat[0:1,1])[0] for dat in x_list]
    points = [ax.plot([dat[0,0]], [dat[0,1]], 'x')[0] for dat in x_list]  
    max_len = 0
    for x in x_list:
        if np.shape(x)[0] > max_len:
            max_len = np.shape(x)[0]

    max_lim = np.zeros((ns,1), dtype = np.double)
    min_lim = np.full((ns,1), np.inf, dtype = np.double)

    for k in range(ns):   
        for x in x_list:     
            max_lim[k] = max(max_lim[k], max(x[:,k]))
            min_lim[k] = min(min_lim[k], min(x[:,k]))
    
    def update_lines(num, dataLines, lines, points) :
        for line, data, pts in zip(lines, dataLines, points) :
            # NOTE: there is no .set_data() for 3 dim data...                                    
            r,_ = np.shape(data)
        
            val = num
            if num > r:
                val = r
            line.set_data( np.transpose(data[:val,0:2]) )            
            pts.set_data( np.transpose(data[val,0:2]) )
        return lines
    

    ax.set_xlim([min_lim[0], max_lim[0]])
    ax.set_xlabel('X')

    ax.set_ylim([min_lim[1], max_lim[1]])
    ax.set_ylabel('Y')


    
    
    ax.set_title('Result animation.')

    # Creating the Animation object
    line_ani = animation.FuncAnimation(fig, update_lines, frames = max_len, fargs=(x_list, lines, points),
                                   interval=1, blit=False)

    if saveVideo:
        from time import gmtime, strftime
        time_str = strftime("%Y-%m-%d-%H-%M-%S", gmtime())
        line_ani.save('Traj_video_'+time_str+'_.mp4', fps=25, \
                        extra_args=['-vcodec', 'libx264'])

    plt.show()
 
    

if __name__ == '__main__':
    
    if len(sys.argv) < 2:
        printUsage()
        
    else:
        dof = sys.argv[1]
        if dof == '2':
            Trajs = readInputFile(ns = 4)
            create_movie_2d(Trajs,saveVideo = False)    
        else:
            Trajs = readInputFile()
            create_movie_3d(Trajs,saveVideo = False)
    
    



