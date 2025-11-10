import json
import random
import uuid
import math

# --- Configuração dos Parâmetros ---

# Define os limites do mundo (de -200 a 200 nos eixos X e Y)
WORLD_BOUNDS = 200.0  # <-- ALTERAÇÃO AQUI (antes era 100.0)

# Número mínimo de waypoints no caminho de cada drone
MIN_WAYPOINTS = 10
# Para adicionar variedade, definimos um máximo de waypoints
MAX_WAYPOINTS = 15 

# Separação mínima entre as posições INICIAIS dos drones
MIN_SEPARATION = 0.2

# Lista de arquivos que queremos gerar e a contagem de drones em cada um
DRONE_COUNTS = [5, 10, 25, 50, 100]

# Intervalo de altitude para os drones
MIN_ALTITUDE = 5.0
MAX_ALTITUDE = 25.0

# --- Funções Auxiliares ---

def generate_uuid_name():
    """Gera um nome de drone no formato 'Drone_' + 8 caracteres hex."""
    return f"Drone_{uuid.uuid4().hex[:8]}"

def generate_waypoint():
    """Gera um único waypoint [x, y] dentro dos limites do mundo."""
    # Esta função agora usará o novo WORLD_BOUNDS (200.0)
    x = random.uniform(-WORLD_BOUNDS, WORLD_BOUNDS)
    y = random.uniform(-WORLD_BOUNDS, WORLD_BOUNDS)
    return [round(x, 3), round(y, 3)]

def calculate_distance(pos1, pos2):
    """Calcula a distância Euclidiana 2D entre duas posições."""
    return math.sqrt((pos2[0] - pos1[0])**2 + (pos2[1] - pos1[1])**2)

def is_position_valid(new_pos, existing_positions):
    """
    Verifica se uma nova posição está a uma distância segura de todas as posições existentes.
    """
    for pos in existing_positions:
        if calculate_distance(new_pos, pos) < MIN_SEPARATION:
            return False
    return True

# --- Função Principal de Geração ---

def generate_drone_data(num_drones):
    """
    Gera uma lista de dicionários de drones, garantindo a separação inicial.
    """
    drone_list = []
    start_positions = []

    print(f"Gerando {num_drones} drones...")

    for i in range(num_drones):
        # --- 1. Encontrar uma Posição Inicial Válida ---
        while True:
            start_pos = generate_waypoint()
            if is_position_valid(start_pos, start_positions):
                start_positions.append(start_pos)
                break
        
        # --- 2. Gerar o Resto do Caminho (Path) ---
        path = [start_pos]
        num_additional_waypoints = random.randint(MIN_WAYPOINTS - 1, MAX_WAYPOINTS - 1)
        
        for _ in range(num_additional_waypoints):
            path.append(generate_waypoint())
            
        # --- 3. Montar o Objeto Drone ---
        drone = {
            "uuid": generate_uuid_name(),
            "altitude": round(random.uniform(MIN_ALTITUDE, MAX_ALTITUDE), 2),
            "path": path
        }
        
        drone_list.append(drone)

    print(f"Sucesso! {num_drones} drones gerados.")
    return drone_list

# --- Execução do Script ---

def main():
    """
    Loop principal que gera todos os arquivos de cenário solicitados.
    """
    for count in DRONE_COUNTS:
        filename = f"path_{count}.json" # Mudei o nome do arquivo
        
        drones = generate_drone_data(count)
        
        try:
            with open(filename, 'w', encoding='utf-8') as f:
                json.dump(drones, f, indent=4)
            print(f"Arquivo '{filename}' salvo com sucesso.\n")
        except IOError as e:
            print(f"Erro ao salvar o arquivo '{filename}': {e}\n")

if __name__ == "__main__":
    main()
