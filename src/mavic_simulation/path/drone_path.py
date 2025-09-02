import tkinter as tk
from tkinter import ttk,  filedialog, messagebox, dialog
import json
import uuid
import util

class DronePathApp:
    def __init__(self, root):
        
        #Parâmetros iniciais da janela
        self.root = root
        self.root.title("Editor de Caminho")
        self.root.geometry("800x800")
        self.root.resizable(False,False)
        self.drones = []
        self.drone_selecionado = None
        
        #função registra a função para validação de valores float
        v_float = (root.register(util.valida_float),'%P')
        
        #definição das divs de conteúdo da janela     
        div_esquerda = ttk.Frame(root,padding= 5, width=20)
        div_esquerda.grid(row=0, column=0, sticky="ew")
        
        div_direita = ttk.Frame(root)
        div_direita.grid(row=0, column=1)     
        
        #definição do conteúdo da div_esquerda, bem como sua posição, usando grid
        
        #Lista de drones
        ttk.Label(div_esquerda, text="Drones").grid(row=0, column=0,columnspan=4)
        self.lista_drones = tk.Listbox(div_esquerda,height=10)
        self.lista_drones.grid(row=1, column=0,columnspan=4,sticky="ew")
        self.lista_drones.bind("<<ListboxSelect>>", self.on_drone_selecionado)
        
        #Botões de adicionar e remover um drone
        ttk.Button(div_esquerda, text="Adicionar Drone", command=self.adiciona_drone).grid(row=2, column =0, columnspan=4, sticky="ew")
        ttk.Button(div_esquerda, text="Remover Drone", command=self.remove_drone).grid(row=3, column = 0, columnspan=4, sticky="ew")
        
        #lista de waypoints
        ttk.Label(div_esquerda, text="Waypoints").grid(row=4, column=0,columnspan=4)
        self.lista_waypoints = tk.Listbox(div_esquerda, height=10)
        self.lista_waypoints.grid(row=5, column=0,columnspan=4, sticky="ew")
        
        #Botões de movimentar os waypoints acima ou abaixo
        ttk.Button(div_esquerda, text="⮝", command=self.move_waypoint_acima).grid(row=6, column=0, columnspan=2, sticky="ew")
        ttk.Button(div_esquerda, text="⮟", command=self.move_waypoint_abaixo).grid(row=6, column=2, columnspan=2,sticky="ew")
        
        #campo de inserção do valor x para um novo waypoint
        ttk.Label(div_esquerda, text="X =").grid(row=7, column=0)
        self.valor_x = ttk.Entry(div_esquerda,validate="all", validatecommand= v_float, width= 5)
        self.valor_x.grid(row=7, column=1)
        
        #campo de inserção do valor y para um novo waypoint
        ttk.Label(div_esquerda, text="Y =").grid(row=7, column=2)
        self.valor_y = ttk.Entry(div_esquerda,validate="all", validatecommand= v_float, width= 5)
        self.valor_y.grid(row=7, column=3)
        
        #botões de adicionar e remover waypoint
        ttk.Button(div_esquerda, text="Adicionar Waypoint", command=self.adiciona_waypoint).grid(row=8,column=0,columnspan=4, sticky="ew")
        ttk.Button(div_esquerda, text="Excluir Waypoint", command=self.remove_waypoint).grid(row=9, column=0, columnspan=4, sticky="ew")
        
        #campos de definição da altitude inicial e final de cada drone.
        self.altitude_inicial = tk.DoubleVar(value=10.0)
        self.altitude_final = tk.DoubleVar(value=20.0)
        ttk.Label(div_esquerda, text="Altitude inicial",justify="left").grid(row=10, column=0, columnspan=3)
        ttk.Entry(div_esquerda, textvariable=self.altitude_inicial, width=7).grid(row=10, column=3)
        ttk.Label(div_esquerda, text="Altitude final").grid(row=11, column=0,columnspan=3)
        ttk.Entry(div_esquerda, textvariable=self.altitude_final, width=7).grid(row=11, column=3)
        ttk.Button(div_esquerda, text="Importar", command=self.importa_json).grid(row=12, column=0,columnspan=2)
        ttk.Button(div_esquerda, text="Exportar", command=self.exporta_json).grid(row=12, column=2,columnspan=2)
        
        
        #canvas, poisicionado na div_direita, onde serão desenhados os drones
        self.canvas = tk.Canvas(div_direita,bg="white", height=580, width= 620)
        self.canvas.grid()
        self.canvas.bind("<Button-1>", self.canvas_click)
        
        #cria um novo drone e o adiciona na lista, com o sufixo Drone_ e um uuid de 8 digítos
    def adiciona_drone(self):
        d = {"uuid": "Drone_"+str(uuid.uuid4())[:8], "path": []}
        self.drones.append(d)
        self.atualiza_lista_drones()

        #remove um drone da lista
    def remove_drone(self):
        idx = self.lista_drones.curselection()
        if not idx:
            return
        self.drones.pop(idx[0])
        self.atualiza_lista_drones()
        self.lista_waypoints.delete(0, tk.END)
        self.drone_selecionado = None
        self.desenha_canvas()

        #evento de seleção de um drone
    def on_drone_selecionado(self, event):
        idx = self.lista_drones.curselection()
        if not idx:
            return
        self.drone_selecionado = self.drones[idx[0]]
        self.atualiza_lista_waypoints()
        self.desenha_canvas()

        #adiciona um novo waypoint, capturando o valor contido nos campos x, e y 
    def adiciona_waypoint(self):
        x = float(self.valor_x.get())
        y = float(self.valor_y.get())
        if not self.drone_selecionado:
            return
        if x is None or y is None:
            return
        self.drone_selecionado["path"].append([x, y])
        self.atualiza_lista_waypoints()
        self.desenha_canvas()

        #remove um waypoint da lista 
    def remove_waypoint(self):
        if not self.drone_selecionado:
            return
        idx = self.lista_waypoints.curselection()
        if not idx:
            return
        self.drone_selecionado["path"].pop(idx[0])
        self.atualiza_lista_waypoints()
        self.desenha_canvas()

        #altera a ordem dos waypoints, movendo o waypoint selecionado uma casa acima
    def move_waypoint_acima(self):
        if not self.drone_selecionado:
            return
        idx = self.lista_waypoints.curselection()
        if not idx or idx[0] == 0:
            return
        i = idx[0]
        self.drone_selecionado["path"][i - 1], self.drone_selecionado["path"][i] = (
            self.drone_selecionado["path"][i],
            self.drone_selecionado["path"][i - 1],
        )
        self.atualiza_lista_waypoints()
        self.lista_waypoints.selection_set(i - 1)
        self.desenha_canvas()

        #altera a ordem dos waypoints, movendo o waypoint selecionado uma casa abaixo
    def move_waypoint_abaixo(self):
        if not self.drone_selecionado:
            return
        idx = self.lista_waypoints.curselection()
        if not idx or idx[0] == len(self.drone_selecionado["path"]) - 1:
            return
        i = idx[0]
        self.drone_selecionado["path"][i + 1], self.drone_selecionado["path"][i] = (
            self.drone_selecionado["path"][i],
            self.drone_selecionado["path"][i + 1],
        )
        self.atualiza_lista_waypoints()
        self.lista_waypoints.selection_set(i + 1)
        self.desenha_canvas()

        #registra o evento de clique no canvas, criando um novo waypoint no local do clique, para o drone selecionado
    def canvas_click(self, event):
        if not self.drone_selecionado:
            return
        x = event.x - 290
        y = 310 - event.y
        self.drone_selecionado["path"].append([x, y])
        self.atualiza_lista_waypoints()
        self.desenha_canvas()

        #atualiza a lista de drones
    def atualiza_lista_drones(self):
        self.lista_drones.delete(0, tk.END)
        for d in self.drones:
            self.lista_drones.insert(tk.END, d["uuid"])

        #atualiza a lista de waypoints
    def atualiza_lista_waypoints(self):
        self.lista_waypoints.delete(0, tk.END)
        for p in self.drone_selecionado["path"]:
            self.lista_waypoints.insert(tk.END, f"({p[0]}, {p[1]})")

        #desenha o canvas, conforme as informações dos drones e waypoints
    def desenha_canvas(self):
        self.canvas.delete("all")
        self.canvas.create_line(290, 0, 290, 580, fill="gray")
        self.canvas.create_line(0, 310, 620, 310, fill="gray")
        colors = ["red", "blue", "green", "orange", "purple"]
        for i, d in enumerate(self.drones):
            color = colors[i % len(colors)]
            pts = d["path"]
            if len(pts) > 1:
                coords = [(290 + x, 310 - y) for x, y in pts]
                self.canvas.create_line(coords, fill=color, width=2)
            for x, y in pts:
                self.canvas.create_oval(
                    290 + x - 3, 310 - y - 3, 290 + x + 3, 310 - y + 3, fill=color
                )
        #importa um arquivo json, contendo drones, waypoints e altitudes, conforme a documentação.
    def importa_json(self):
        path = filedialog.askopenfilename(defaultextension="json", initialdir=".")
        with open(path) as drones_path_file:
            drones = json.load(drones_path_file)
        self.drones = drones
        self.drone_selecionado = self.drones[0]
        self.atualiza_lista_drones()
        self.atualiza_lista_waypoints()
        self.desenha_canvas()
        
        #exporta um arquivo json, contendo drones, waypoints e altitudes, conforme a documentação
    def exporta_json(self):
        if not self.drones:
            messagebox.showerror("Erro", "Nenhum drone definido.")
            return
        alt1, alt2 = self.altitude_inicial.get(), self.altitude_final.get()
        n = len(self.drones)
        exported = []
        for i, d in enumerate(self.drones):
            alt = alt1 + (alt2 - alt1) * (i / max(1, n - 1))
            exported.append({"uuid": d["uuid"], "altitude": alt, "path": d["path"]})
        path = filedialog.asksaveasfilename(defaultextension=".json", initialfile=str(uuid.uuid4())+".json")
        if not path:
            return
        with open(path, "w", encoding="utf-8") as f:
            json.dump(exported, f, indent=4)
        messagebox.showinfo("Sucesso", "Arquivo exportado.")


if __name__ == "__main__":
    root = tk.Tk()
    app = DronePathApp(root)
    root.mainloop()
