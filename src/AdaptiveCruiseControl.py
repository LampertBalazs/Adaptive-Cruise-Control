import matplotlib.pyplot as plt
import numpy as np


class Vehiclemodell:

    def __init__(self, m=1500, Cd=0.3, A=2.2, rho=1.2, Cr=0.015, x0=0.0,v0=20.0):

        self.m=m  #járműtömeg
        self.Cd= Cd #Légellenállás
        self.A=A #homlokfelület
        self.rho=rho #levegő sűrűség
        self.Cr =Cr #gördülőellenállás
        self.g=9.81 #Navajonmi?

        self.x=x0 #pozíció
        self.v=v0 #sebesség


#F_h hajtóerő valamilyen motorkarakterisztika kell ebben van a "gázpedálállás" bemenetként (tempomat szóval nem tényleges gázpedálállás de aként lehet felfogni a controller felől nézve)
#F_f fékerő, valami reális szintén modell alapján "Fékpedálállás" bemented zwei (tempomat szóval nem tényleges fékpedálállás de aként lehet felfogni a controller felől nézve)
#erők számítása modellek alapján (Légellenállás gördülőellenállás)
    def step(self, F_h, F_f, dt=0.01):

        F_légell=0.5*self.rho*self.Cd*self.A*(self.v**2)
        F_gördell=self.m*self.g*self.Cr

        F_össz=F_h-F_f-F_gördell-F_légell

        #gyorsulás
        a=F_össz/self.m

        #Állapotok frissítése (gyakorlatilag a differenciálegyenlet megoldása)
        self.x +=self.v*dt +0.5*a*(dt**2)
        self.v +=a*dt

        # Ha ez nincs tolatni fog, de nelünk az nyem jó
        if self.v < 0:
            self.v = 0
        return self.x, self.v


#szimulációs paraméterek
dt=0.05
t_szimidő=40.00
ido=np.arange(0,t_szimidő,dt)
steps=len(ido)

#initializálás
host= Vehiclemodell(x0=0.0,v0=20.0)
vasalóswift_x=80.0
vasalóswift_v=20.0

d0=10.0 #követési táv állva
T_köv=1.5 #követési idő 
Kp=600 #P tag PID
Kd=300 #D tag pid

#adattároló
data={'ido':[], 'host_x':[], 'host_v':[], 'vasalóswift_x':[], 'vasalóswift_v':[], 'distance':[], 'desired_distance':[]}


#szimuláció
for t in ido:

    if t>15.0:
        vasalóswift_v=max(12.0,vasalóswift_v-0.5*dt) #lassítás
        vasalóswift_x+=vasalóswift_v*dt


    #előző állapotok
    host_x=host.x
    host_v=host.v

    #követési táv számítása
    d_köv=d0+T_köv*host_v

    #hibaszámítás
    hiba=(vasalóswift_x-host_x)-d_köv

    #sebességkülönbség számítása
    delta_v=vasalóswift_v-host_v

    #PID szabályozó (P+D)
    F_h=Kp*hiba+Kd*delta_v
    F_f=0.0

    #modell léptetése
    host_x, host_v=host.step(F_h,F_f,dt)

    #adatok tárolása
    data['ido'].append(t)
    data['host_x'].append(host_x)
    data['host_v'].append(host_v)
    data['vasalóswift_x'].append(vasalóswift_x)
    data['vasalóswift_v'].append(vasalóswift_v)
    data['distance'].append(vasalóswift_x - host_x)
    data['desired_distance'].append(d_köv)


plt.figure(figsize=(10,10))
plt.subplot(2,1,1)
plt.plot(data['ido'],data['host_v'],label="Host velocity")
plt.plot(data['ido'],data['vasalóswift_v'],label="Vasalóswift velocity")
plt.xlabel("Time [s]")
plt.ylabel("Velocity [m/s]")
plt.legend()
plt.title("Adaptive Cruise Control Simulation")
plt.grid(True)

plt.subplot(2,1,2)
plt.plot(data['ido'],data['distance'],label="Actual Distance")
plt.plot(data['ido'],data['desired_distance'],label="Desired Distance")
plt.xlabel("Time [s]")
plt.ylabel("Distance [m]")
plt.legend()
plt.title("Distance Control")
plt.grid(True)

plt.tight_layout()
plt.show()
