#importando Librerias

from cProfile import label
import streamlit as st
import pandas as pd
import numpy as np
from sklearn import preprocessing, cluster
import scipy
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
from haversine import haversine, Unit
import folium
from folium import features
from streamlit_folium import folium_static
from PIL import Image
import time

def importar_datos():
    """ Esta Funcion Importa los datos para el Data Frame"""
    path = 'inegi_jalisco_prueba.csv'
    df = pd.read_csv(path, encoding='ISO-8859-1')
    ## Copy of Original DataFrame with selected columns only
    df = df[['nom_estab','nombre_act','nom_vial','nom_v_e_1','nom_v_e_2','numero_ext','nomb_asent',
                        'municipio','latitud','longitud']].copy()
    ## Rename Columns Names
    df.rename({'nom_estab':'Establecimiento', 'nombre_act':'Actividad', 'nom_vial':'Calle', 'nom_v_e_1':'Cruce1', 'nom_v_e_2':'Cruce2',
        'nomb_asent':'Colonia'},axis=1,inplace=True)
    return df    

def seleccionar_municipio(municipio):
    ## Selecting Zapopan to work with this data only and filtering by Activities that include Abarrotes  
    df_municipio = df.loc[df['municipio']==municipio]
    # df_municipio = df_municipio.loc[(df_municipio['Actividad']=='Comercio al por mayor de abarrotes') | (df_municipio['Actividad']=='Comercio al por menor en tiendas de abarrotes, ultramarinos y misceláneas')]
    # df_zapopan['numero_ext'] = df_zapopan['numero_ext'].astype('int')
    df_municipio['numero_ext'] = df_municipio['numero_ext'].fillna(0)
    df_municipio['numero_ext'] = df_municipio['numero_ext'].astype('int')  
    ## Checking Zapopan´s DataFrame information (Number of records and Data Types)
    return df_municipio

# Calling the first function to import the Data from csv file
datos_df = importar_datos()
df = datos_df
municipios = sorted(df['municipio'].unique())

# Streamlit iniziation
st.set_page_config(page_title='Rutas Última Milla',layout='centered',page_icon=':car:')
st.title('Rutas Ultima Milla')
image = Image.open('Logistica.PNG')
st.image(image, caption='Soluciones Última Milla')

st.sidebar.subheader('Indique Latitud y Longitud de su CEDIS')      
latitud = st.sidebar.number_input(label='Latitud',value=20.6744485)
longitud = st.sidebar.number_input(label='Longitud',value=-103.3873984)

st.sidebar.subheader('Seleccione un Municipio')
municipio_st = st.sidebar.selectbox(label='',options=municipios,index=0)

cargando = st.sidebar.empty()
my_bar = st.sidebar.progress(0)
for percent_complete in range(101):
    time.sleep(0.1)
    cargando.text('{}%' .format(percent_complete))
    my_bar.progress(value=percent_complete)
    
# Function to create the distances between points
def crear_distancias(data):
        for n in range(len(data)):
            for i in range(len(data)):
                if n!=i :
                    distancia.append(haversine((data.iloc[n][0],data.iloc[n][1]),(data.iloc[i][0],data.iloc[i][1])))

# Function to create customers 1 by each coordinate
def crear_clientes(data):
    for i in range(len(data)):
        clientes.append(i)

# Function to create the Arcs between customers
def crear_arcos(data):        
        for i in clientes:
            for j in clientes:
                if i!=j:
                    arcos.append((i,j))

# Function to create the Matrix with the distances
def crear_matriz(data):
    filas = len(data)
    columnas = len(data)
    contador = 0
    for i in range(filas):
        matriz.append([])
        for j in range(columnas):
            if i == j:
                matriz[i].append(0)
            else:
                matriz[i].append(distancia[contador])
                contador += 1

# calling the functions to start the routes analysis
df_municipio = seleccionar_municipio(municipio_st)
cedis = [latitud,longitud]
df = df_municipio.loc[:,['latitud','longitud']]
df.iloc[0] = cedis
df1 = df.iloc[:6]
matriz = []
clientes = []
arcos=[]
distancia = []
crear_clientes(df1)
crear_arcos(df1)
crear_distancias(df1)
crear_matriz(clientes)
matriz = pd.DataFrame(matriz)
matriz = np.array(matriz)

# Transform to Numpy Array and Scale the Distances
distance_matrix = np.round(matriz * 1000)

# Create Data Model
def create_data_model():
    """Stores the data for the problem."""
    data = {}
    data['distance_matrix'] = distance_matrix
    data['num_vehicles'] = 1
    data['depot'] = 0
    return data

# Print solution on console
def print_solution(manager, routing, solution):
    """Prints solution on console."""
    print('Objective: {} Kms'.format(solution.ObjectiveValue()/1000))
    index = routing.Start(0)
    plan_output = 'Ruta propuesta:\n'
    route_distance = 0
    nodos = []
    while not routing.IsEnd(index): 
        nodos.append(manager.IndexToNode(index))
        plan_output += ' {} ->'.format(manager.IndexToNode(index))
        previous_index = index
        index = solution.Value(routing.NextVar(index))
        route_distance += routing.GetArcCostForVehicle(previous_index, index, 0)
    plan_output += ' {}\n'.format(manager.IndexToNode(index))
    plan_output += ', Distancia de recorrido: {}Kms\n'.format(route_distance/1000)
    nodos.append(0)
    st.text('Expanda las secciones abajo para ver la Ruta popuesta para el recorrido y el Mapa')
    with st.expander('Ruta Propuesta'):
            st.write(plan_output)
    print('Nodos: ' + str(nodos))
    return nodos

# Solving the problem
data = create_data_model()
# Create the routing index manager.
manager = pywrapcp.RoutingIndexManager(len(data['distance_matrix']),
                                   data['num_vehicles'], data['depot'])

# Create Routing Model
routing = pywrapcp.RoutingModel(manager)

def distance_callback(from_index, to_index):
    """Returns the distance between the two nodes."""
    # Convert from routing variable Index to distance matrix NodeIndex.
    from_node = manager.IndexToNode(from_index)
    to_node = manager.IndexToNode(to_index)
    return data['distance_matrix'][from_node][to_node]

# Create and register a transit callback.
transit_callback_index = routing.RegisterTransitCallback(distance_callback)
 # Define cost of each arc.
routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

# Setting first solution heuristic.
search_parameters = pywrapcp.DefaultRoutingSearchParameters()
search_parameters.first_solution_strategy = (
    routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

# Solve the problem.
solution = routing.SolveWithParameters(search_parameters)

if solution:
    nodos = print_solution(manager,routing,solution)
else:
    print('No Solution')    

# Visualization on Map
puntos = [cedis]
for n in nodos:
    if n != 0:
        puntos.append([df_municipio.iloc[n][8],df_municipio.iloc[n][9]])

# linea = folium.PolyLine(puntos)
poligono = folium.Polygon(puntos)

m = folium.Map(location=[latitud, longitud],zoom_start=8)
# seleccion = print_solution(manager, routing, solution)
# nodos = seleccion

cont=0
for n in puntos:
    if n == cedis:
        marcador = folium.Marker(location=n,popup='Cedis',icon=folium.Icon(color="green", icon="cloud"))
        m.add_child(marcador)
    else:    
        marcador = folium.Marker(location=n,popup=nodos[cont],icon=folium.Icon(color="red", icon="info-sign"))
        m.add_child(marcador)       
    cont += 1
            
m.add_child(poligono)


with st.expander(label='Ver mapa de ruta Propuesta'):
    mapa = folium_static(m)



