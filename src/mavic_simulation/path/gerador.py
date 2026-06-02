import json
import random
import uuid
import math

# --- Configuração dos Parâmetros ---

# Define os limites do mundo (de -200 a 200 nos eixos X e Y)
WORLD_BOUNDS = 200.0  

# Número mínimo e máximo de waypoints no caminho de cada drone (ALTERADO AQUI)
MIN_WAYPOINTS = 5
MAX_WAYPOINTS = 10 

# Separação mínima entre as posições INICIAIS dos drones
MIN_SEPARATION = 0.2

# Lista de arquivos que queremos gerar e a contagem de drones em cada um
DRONE_COUNTS = [5, 10, 25, 50, 100]

# Intervalo de altitude para os drones
MIN_ALTITUDE = 5.0
MAX_ALTITUDE = 25.0

# --- PARÂMETROS DE DISTÂNCIA ---
# Distância máxima total que o drone pode percorrer (1km = 1000 unidades/metros)
MAX_TOTAL_DISTANCE = 1000.0
# Distância máxima por segmento (salto entre waypoints) para distribuir o movimento
MAX_SEGMENT_DISTANCE = 100.0

# --- Funções Auxiliares ---

def generate_uuid_name():
    """Gera um nome de drone no formato 'Drone_' + 8 caracteres hex."""
    return f"Drone_{uuid.uuid4().hex[:8]}"

def generate_random_position():
    """Gera uma posição base [x, y] completamente aleatória dentro dos limites."""
    x = random.uniform(-WORLD_BOUNDS, WORLD_BOUNDS)
    y = random.uniform(-WORLD_BOUNDS, WORLD_BOUNDS)
    return [round(x, 3), round(y, 3)]

def calculate_distance(pos1, pos2):
    """Calcula a distância Euclidiana 2D entre duas posições."""
    return math.sqrt((pos2[0] - pos1[0])**2 + (pos2[1] - pos1[1])**2)

def is_position_valid(new_pos, existing_positions):
    """Verifica se uma nova posição inicial está a uma distância segura das existentes."""
    for pos in existing_positions:
        if calculate_distance(new_pos, pos) < MIN_SEPARATION:
            return False
    return True

def generate_next_waypoint(current_pos, remaining_dist):
    """
    Gera o próximo waypoint movendo-se na direção de um alvo aleatório,
    mas limitando o tamanho do passo para não exceder o limite restante de 1km.
    """
    target_pos = generate_random_position()
    dist = calculate_distance(current_pos, target_pos)
    
    if dist == 0:
        return current_pos, 0.0
        
    # Define o quanto o drone vai andar: pega o menor valor entre a distância até o alvo,
    # o limite máximo do segmento e a distância que ainda resta no 'orçamento' de 1km.
    allowed_dist = min(dist, remaining_dist, MAX_SEGMENT_DISTANCE)
    
    # Interpolação linear para encontrar as exatas coordenadas do novo ponto
    ratio = allowed_dist / dist
    new_x = current_pos[0] + (target_pos[0] - current_pos[0]) * ratio
    new_y = current_pos[1] + (target_pos[1] - current_pos[1]) * ratio
    
    # Prevenção extra para garantir que o drone não fure os limites mundiais
    new_x = max(-WORLD_BOUNDS, min(WORLD_BOUNDS, new_x))
    new_y = max(-WORLD_BOUNDS, min(WORLD_BOUNDS, new_y))
    
    return [round(new_x, 3), round(new_y, 3)], allowed_dist

# --- Função Principal de Geração ---

def generate_drone_data(num_drones):
    """Gera uma lista de dicionários de drones, garantindo a separação e distância máxima de 1km."""
    drone_list = []
    start_positions = []

    print(f"Gerando {num_drones} drones...")

    for i in range(num_drones):
        # --- 1. Encontrar uma Posição Inicial Válida ---
        while True:
            start_pos = generate_random_position()
            if is_position_valid(start_pos, start_positions):
                start_positions.append(start_pos)
                break
        
        # --- 2. Gerar o Resto do Caminho (Path) ---
        path = [start_pos]
        num_additional_waypoints = random.randint(MIN_WAYPOINTS - 1, MAX_WAYPOINTS - 1)
        
        current_pos = start_pos
        total_distance_travelled = 0.0
        
        for _ in range(num_additional_waypoints):
            remaining_dist = MAX_TOTAL_DISTANCE - total_distance_travelled
            
            # Se o drone já voou 1km, ele passa a pairar no lugar para preencher a cota de waypoints
            if remaining_dist <= 0:
                path.append(current_pos)
                continue
                
            next_pos, step_dist = generate_next_waypoint(current_pos, remaining_dist)
            path.append(next_pos)
            
            current_pos = next_pos
            total_distance_travelled += step_dist
            
        # --- 3. Montar o Objeto Drone ---
        drone = {
            "uuid": generate_uuid_name(),
            "altitude": round(random.uniform(MIN_ALTITUDE, MAX_ALTITUDE), 2),
            "path": path,
            "total_distance": round(total_distance_travelled, 2) 
        }
        
        drone_list.append(drone)

    print(f"Sucesso! {num_drones} drones gerados.")
    return drone_list

# --- Execução do Script ---

def main():
    """Loop principal que gera todos os arquivos de cenário solicitados."""
    for count in DRONE_COUNTS:
        filename = f"path_{count}.json" 
        
        drones = generate_drone_data(count)
        
        try:
            with open(filename, 'w', encoding='utf-8') as f:
                json.dump(drones, f, indent=4)
            print(f"Arquivo '{filename}' salvo com sucesso.\n")
        except IOError as e:
            print(f"Erro ao salvar o arquivo '{filename}': {e}\n")

if __name__ == "__main__":
    main()
