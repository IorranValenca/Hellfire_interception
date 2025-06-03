import numpy as np
import matplotlib.pyplot as plt
#Funçao auxiliar para calcular a densidade do ar
# Simulador de impacto de um AGM 114 Hellfire
def air_density(z):
    return 1.225 * np.exp(-z / 8.5)

def norm(v):
    return np.sqrt(np.sum(v**2))


#Posiçao alvo
target_pos = np.array([0.0, 0.0, 0.0])
#velocidade do alvo
target_vel = np.array([60.0, 0.0, 0.0])

#Posiçao do missel
missile_pos = np.array([3000.0, 200.0, 0.0]) #Saindo de um Jato 
missile_vel = np.array([-100.0, -50.0, 0.0]) #Velocidade inicial do missel

#Informações do missel
mass = 45 #massa em kg do AGM 114 Hellfire
Cd = 0.15 #Coeficiente de arrasto
A = 0.015 #Área frontal do missel
thrust = 5000.0 #Empuxo do propulsor do  missel em N
burn_time = 2.0 #Tempo de queima do propulsor em segundos ( 2 primeiros segundos ajustando a velocidade do missel)
g = 9.81 #Aceleração da gravidade em m/s^2
dt = 0.01 #Passo de tempo em segundos
t= 0.0 #Tempo inicial


#NAvegaçao proporcional (Ganho proporcional)
N = 3.0

#Plot do cenário

T, TX, TY, TZ = [], [], [], []
MX, MY, MZ = [], [], []

while t < 30:
    #Save posiçoes pro plot
    T.append(t)
    TX.append(target_pos[0])
    TY.append(target_pos[1])
    TZ.append(target_pos[2])
    MX.append(missile_pos[0])
    MY.append(missile_pos[1])
    MZ.append(missile_pos[2])

    #funçao pra atualizar a posiçao do alvo
    target_pos += target_vel * dt
    
    # VEtor linha de visada
    los = target_pos - missile_pos
    los_norm= norm(los)

    if los_norm < 5: 
        print(f"Alvo neutralizado em {t:.2f} segundos")
        break

    los_unit = los / los_norm

    #V relativa do missel em relaçao ao alvo
    rel_vel = target_vel - missile_vel
    omega = np.cross(los, rel_vel) / (los_norm**2 + 1e-6) # Vetor ortogonal ao vetor de linha de visada e a velocidade relativa

    #COrreção do rumo do missel 
    accel_cmd = N * np.cross(omega, missile_vel) #Comando de aceleração proporcional

    #Empuxo dos 2 segundos
    if t < burn_time:
        thrust_vec = thrust * los_unit
    else:
        thrust_vec = np.array([0.0, 0.0, 0.0]) #Sem empuxo após o tempo de queima

    #Força de total
    v_m = norm(missile_vel)
    rho = air_density(missile_pos[2]) #Densidade do ar
    drag = 0.5 * Cd *rho * A * v_m**2 #Força de arrasto
    drag_vec = -drag * missile_vel / (v_m + 1e-6)#Força de arrasto

    total_force = thrust_vec + accel_cmd + drag_vec - np.array([0.0, 0.0, mass * g]) #Força total
    accel = total_force / mass #Aceleração do missel formula F = m * a = força total dividido pela massa

    # Update missile physics (THIS WAS MISSING!)
    missile_vel += accel * dt
    missile_pos += missile_vel * dt

    t += dt #Incrementa o tempo


#Plot 2D
plt.figure(figsize=(10, 6))
plt.plot(TX, TY, label='Alvo', color='blue')
plt.plot(MX, MY, label='Interceptador (IA PN)', color='red')
plt.xlabel('X (m)')
plt.ylabel('Y (m)') 
plt.title('Simulação de Impacto do AGM 114 Hellfire com Navegação Proporcional')
plt.legend()
plt.grid(True)  # Changed from "True" to True
plt.axis('equal')
plt.show()
