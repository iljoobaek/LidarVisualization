import numpy as np
velo = np.fromfile("image.bin",dtype =np.float32).reshape(-1,4)
#velo = np.fromfile("image.bin")
#print(min(velo))
for vel in velo:
    print (vel[3])
'''
vel_dum_x = []
vel_dum_y = []
vel_dum_z = []
count = 0
for vel in velo:
    if vel[0] in vel_dum_x and vel[1] in vel_dum_y and vel[2] in vel_dum_z:
        if abs(int(vel[0])) is not 0 and abs(int(vel[1])) is not 0:
            print(vel[2])
            print(vel_dum_z[vel_dum_x.index(vel[0])])
            count = count + 1
    vel_dum_x.append(vel[0])
    vel_dum_y.append(vel[1])
    vel_dum_z.append(vel[2])
print(count)'''
