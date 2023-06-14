# State2K.py

import sys
sys.path.append(r'C:\Users\ignac\OneDrive - Universidad Politécnica de Madrid\04 - GIA 4 - Cuatri 8\TFG\TFG-AI-repo\Networks')
sys.path.append(r'C:\Users\ignac\OneDrive - Universidad Politécnica de Madrid\04 - GIA 4 - Cuatri 8\TFG\TFG-AI-repo\Models')

import torch
import utils
import numpy as np

# MODEL state_dict PATH:
dict_path = '../Models/'
# reference state_dict name for the model.
# call the desired path when giving birth to the network
model_name = 'state_dict_002_A'


def model_birth(model_name=model_name):
    """Creates a network from a saved `state_dict` in `..\Models\` with the name `model_name`."""
    # network structure parameters:
    input_size = 2
    output_size = 10
    hidden_layers = [20, 20]
    # network initialization:
    model = utils.NetworkReduced(input_size=input_size, output_size=output_size, hidden_layers=hidden_layers)
    # network loading
    model.load_state_dict(torch.load(dict_path + model_name))
    # setting the network in evaluation mode (setting dropout and batch normalization layers to evaluation mode)
    model.eval()
    return model

def InfereController(model, state):
    K = utils.PredictController(model, state)
    return K



from joblib import load

# scalerX:
# path:
scaler_path = '../Models/'
# TO CHANGE: name of the one to use:
scaler_name = 'scalerX_002.bin'
# Loading reference scaler:
scalerX = load(scaler_path + scaler_name)

def PredictController2(model, state, scalerX=scalerX):
    """
    Function that receives the trim state vector and generates a prediction of the appropriate
    controller based on the trained model 'model'.

    Arguments
    ---------
    model: Network to use
    state: State in the flight envelope to infere from
    scalerX: scaler used for the features list from the dataset
    """
    # 
    state = np.array(state).reshape(1,-1)
    features = scalerX.transform(state)
    out_pred = model(torch.Tensor(features))    # torch.Tensor([1,10])
    K_comps = out_pred.detach().numpy()
    state_dim = K_comps.size    # 10
    return K_comps.reshape(2, int(state_dim/2))


# To call 'model' from Matlab:
model = model_birth(model_name)

# To call 'K' from Matlab:
state_ref = [20., 2000.]
K = InfereController(model, state=state_ref)
# print(K)

K2 = PredictController2(model, state=state_ref)