import numpy as np

grid = [[0, 0, 1, 0, 0, 0],
        [0, 0, 1, 0, 0, 0],
        [0, 0, 0, 0, 1, 0],
        [0, 0, 1, 1, 1, 0],
        [0, 0, 0, 0, 1, 0]]

init = [0, 0]
goal = [len(grid)-1, len(grid[0])-1]
cost = 1

npgrid = np.array(grid)
npg = np.zeros(npgrid.shape)
#print(npgrid)

def search(grid,init,goal,cost):
    path =1
    gscores(npgrid,goal)
    return path

def gscores(npg,goal):
    #pad with 1s
    r,c = npg.shape
    npones = np.ones((r+2,c+2))*-1

    npones[1:r+1,1:c+1] = npg
    npg = npones
    gr = goal[0]+1
    gc = goal[1]+1
    npg[gr,gc] = -2



    gr = goal[0]+1
    gc = goal[1]+1

    map_horz(gr,gc,1,npg)
    gr = goal[0]+1
    gc = goal[1]+1
    map_horz(gr,gc,-1,npg)

    grh =1
    gch =1

    while gch < npg.shape[1]:
        grh, gch = find_value(grh,gch,npg)
        map_vert(grh, gch,1,npg)
        map_vert(grh, gch,-1,npg)
        gch = gch +1
    npg[goal[0]+1,goal[1]+1] =0 # reset goal to 0


    #fill holes
    npwalls = np.array(grid)
    npones = np.ones((r+2,c+2))*1
    npones[1:r+1,1:c+1] = npwalls
    npwalls = npones
    rw,rc =  npwalls.shape
    npwalls[goal[0]+1,goal[1]+1] = -1
    npwalls[init[0]+1,init[1]+1] = -1
    break_me = True
    while break_me:
        break_me = False
        for c in range (1,rc):
            for r in range (1,rw):
                if r == 5 and  c == 4 :
                    print("target")
                if npwalls[r,c] == 0:
                    wallvalue = npwalls[r-1,c]+npwalls[r+1,c]+ npwalls[r,c-1]+ npwalls[r,c+1]
                    if (wallvalue == 3):
                        npwalls[r,c] = 1
                        break_me = True
                r = r+ 1
            r = 1
            c = c + 1
    print(npg)
    print("")
    print(npwalls)








#def fill_in:



def find_value(grh,gch,npg):
    while True:
        if npg[grh,gch] ==0:
            grh = grh  +1
        else:break
    return grh, gch


def map_horz(gr,gc,step,npg):
    while npg[gr,gc+step] != -1:

        if npg[gr,gc+step] !=-1:
            npg[gr,gc+step] = calc_value(gr,gc,npg)
            gc = gc + step


def map_vert(gr,gc,step,npg):
    while npg[gr+step,gc] != -1:

        if npg[gr+step,gc] !=-1:
            npg[gr+step,gc] = calc_value(gr,gc,npg)
            gr = gr+ step


def calc_value(gr,gc,npg):
    if npg[gr,gc] == -2:
        value = 1
    else: value = npg[gr,gc]  +1
    return value


gscores(npg,goal)


