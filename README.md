# proyecto-computacion-cuantica
!pip install cirq matplotlib
import cirq
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Función para graficar la esfera de Bloch con ejes X, Y, Z y contornos
def plot_bloch_vector(state, title="Esfera de Bloch"):
    fig = plt.figure(figsize=(8,8))
    ax = fig.add_subplot(111, projection='3d')

    # Graficar la esfera de Bloch con contornos
    u = np.linspace(0, 2 * np.pi, 100)
    v = np.linspace(0, np.pi, 100)
    x = np.outer(np.cos(u), np.sin(v))
    y = np.outer(np.sin(u), np.sin(v))
    z = np.outer(np.ones(np.size(u)), np.cos(v))

    # Dibuja la malla de la esfera (contornos)
    ax.plot_wireframe(x, y, z, color='b', alpha=0.2)

    # Graficar los ejes X, Y, Z
    ax.quiver(0, 0, 0, 1, 0, 0, color='r', label="Eje X", lw=2)
    ax.quiver(0, 0, 0, 0, 1, 0, color='g', label="Eje Y", lw=2)
    ax.quiver(0, 0, 0, 0, 0, 1, color='b', label="Eje Z", lw=2)

    # Transformamos el estado del qubit a coordenadas 3D
    theta = 2 * np.arccos(np.abs(state[0]))  # Ángulo polar
    phi = np.angle(state[1]) - np.angle(state[0])  # Ángulo azimutal

    # Calculamos las coordenadas 3D del vector de Bloch
    x_bloch = np.sin(theta) * np.cos(phi)
    y_bloch = np.sin(theta) * np.sin(phi)
    z_bloch = np.cos(theta)

    # Graficamos el vector del estado sobre la esfera
    ax.quiver(0, 0, 0, x_bloch, y_bloch, z_bloch, length=1.0, color='k', linewidth=3)

    # Establecer los límites y las etiquetas de los ejes
    ax.set_xlim([-1, 1])
    ax.set_ylim([-1, 1])
    ax.set_zlim([-1, 1])
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    # Título y vista
    ax.set_title(title)
    ax.view_init(30, 45)  # Establecer la vista 3D

    plt.legend()
    plt.show()

# Crear un qubit
qubit = cirq.LineQubit(0)

# Crear el circuito para el estado |1⟩ (aplicamos X a |0⟩)
circuit_initial = cirq.Circuit(
    cirq.X(qubit)  # Aplica X para llevar el qubit a |1⟩
)

# Simular el circuito inicial (estado |1⟩)
simulator = cirq.Simulator()
result_initial = simulator.simulate(circuit_initial)
state_initial = result_initial.state_vector()

# Aplicar la compuerta Hadamard (H) para crear una superposición
circuit_hadamard = cirq.Circuit(
    cirq.X(qubit),  # Llevar a |1⟩
    cirq.H(qubit)   # Aplicar la compuerta Hadamard (H)
)

# Simular el circuito con Hadamard
result_hadamard = simulator.simulate(circuit_hadamard)
state_hadamard = result_hadamard.state_vector()

# Aplicar la compuerta de fase S al estado con Hadamard
circuit_final = cirq.Circuit(
    cirq.X(qubit),  # Llevar a |1⟩
    cirq.H(qubit),  # Aplicar Hadamard
    cirq.S(qubit)   # Aplicar la compuerta de fase S
)

# Simular el circuito final (estado afectado por S después de Hadamard)
result_final = simulator.simulate(circuit_final)
state_final = result_final.state_vector()

# Graficar el estado inicial, después de Hadamard y con la fase S
plot_bloch_vector(state_initial, "Estado Inicial |1⟩")
plot_bloch_vector(state_hadamard, "Estado después de Hadamard (|+⟩)")
plot_bloch_vector(state_final, "Estado después de Hadamard + S")

# Mostrar los circuitos
print("Circuito Inicial (|1⟩):")
print(circuit_initial)

print("Circuito con Hadamard:")
print(circuit_hadamard)

print("Circuito con Hadamard + S:")
print(circuit_final)

import cirq
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Función para graficar la esfera de Bloch con ejes X, Y, Z y contornos
def plot_bloch_vector(state, title="Esfera de Bloch"):
    fig = plt.figure(figsize=(8,8))
    ax = fig.add_subplot(111, projection='3d')

    # Graficar la esfera de Bloch con contornos
    u = np.linspace(0, 2 * np.pi, 100)
    v = np.linspace(0, np.pi, 100)
    x = np.outer(np.cos(u), np.sin(v))
    y = np.outer(np.sin(u), np.sin(v))
    z = np.outer(np.ones(np.size(u)), np.cos(v))

    # Dibuja la malla de la esfera (contornos)
    ax.plot_wireframe(x, y, z, color='b', alpha=0.2)

    # Graficar los ejes X, Y, Z
    ax.quiver(0, 0, 0, 1, 0, 0, color='r', label="Eje X", lw=2)
    ax.quiver(0, 0, 0, 0, 1, 0, color='g', label="Eje Y", lw=2)
    ax.quiver(0, 0, 0, 0, 0, 1, color='b', label="Eje Z", lw=2)

    # Transformamos el estado del qubit a coordenadas 3D
    # El vector de estado de un qubit tiene dos componentes, a_0 y a_1
    # En la esfera de Bloch, mapeamos estos valores a coordenadas 3D
    theta = 2 * np.arccos(np.abs(state[0]))  # Ángulo polar
    phi = np.angle(state[1]) - np.angle(state[0])  # Ángulo azimutal

    # Calculamos las coordenadas 3D del vector de Bloch
    x_bloch = np.sin(theta) * np.cos(phi)
    y_bloch = np.sin(theta) * np.sin(phi)
    z_bloch = np.cos(theta)

    # Graficamos el vector del estado sobre la esfera
    ax.quiver(0, 0, 0, x_bloch, y_bloch, z_bloch, length=1.0, color='k', linewidth=3)

    # Establecer los límites y las etiquetas de los ejes
    ax.set_xlim([-1, 1])
    ax.set_ylim([-1, 1])
    ax.set_zlim([-1, 1])
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    # Título y vista
    ax.set_title(title)
    ax.view_init(30, 45)  # Establecer la vista 3D

    plt.legend()
    plt.show()

# Crear un qubit
qubit = cirq.LineQubit(0)

# Crear el circuito para el estado |1⟩ (aplicamos X a |0⟩)
circuit_initial = cirq.Circuit(
    cirq.X(qubit)  # Aplica X para llevar el qubit a |1⟩
)

# Simular el circuito inicial (estado |1⟩)
simulator = cirq.Simulator()
result_initial = simulator.simulate(circuit_initial)
state_initial = result_initial.state_vector()

# Aplicar la compuerta de fase S
circuit_final = cirq.Circuit(
    cirq.X(qubit),  # Llevar a |1⟩
    cirq.S(qubit)   # Aplicar la compuerta de fase S
)

# Simular el circuito final (estado afectado por S)
result_final = simulator.simulate(circuit_final)
state_final = result_final.state_vector()

# Graficar el estado inicial y final en la esfera de Bloch
plot_bloch_vector(state_initial, "Estado Inicial |1⟩")
plot_bloch_vector(state_final, "Estado después de la compuerta de fase S")

# Mostrar los circuitos
print("Circuito Inicial (|1⟩):")
print(circuit_initial)

print("Circuito Final (con compuerta S):")
print(circuit_final)
