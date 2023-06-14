# state2k.py

# PACKAGES
import sys
sys.path.append(r'C:\Users\ignac\OneDrive - Universidad Politécnica de Madrid\04 - GIA 4 - Cuatri 8\TFG\TFG-AI-repo\Networks')
sys.path.append(r'C:\Users\ignac\OneDrive - Universidad Politécnica de Madrid\04 - GIA 4 - Cuatri 8\TFG\TFG-AI-repo\Models')

# data handling
import numpy as np
from sklearn.preprocessing import StandardScaler

# deep learning
import torch
import torch.nn as nn
import torch.nn.functional as F

# to load the loading the Standard Scaler
from joblib import load

# models path (for state_dict, structure, and scalerX) and general strings:
models_path = '../Models/'
root_name_given_to_state_dict = 'state_dict_'
root_name_given_to_structure = 'structure_'
root_name_given_to_scalerX = 'scalerX_'

# ----- IDENTIFIER ------
identifier = '009_20_40_40_20'
# ------ -------- -------

# default structure:
structure = load(models_path + root_name_given_to_structure + identifier)
# default scalerX:
scalerX = load(models_path + root_name_given_to_scalerX + identifier[0:3] + '.bin')


##################################################################################################
##################################################################################################


'''Only the forward method is included in this reduced network.'''
class NetworkReduced(nn.Module):
    def __init__(self, input_size, output_size, hidden_layers):
        ''' Builds a fully connected network with arbitrary hidden layers.
            
            Arguments
            ---------
            input_size: integer, size of the input layer.
            output_size: integer, size of the output layer.
            hidden_layers: list of integers, the sizes of the hidden layers.'''
        
        super(NetworkReduced, self).__init__()
        # Input to a hidden layer
        self.hidden_layers = nn.ModuleList([nn.Linear(input_size, hidden_layers[0])])
        # Add a variable number of more hidden layers
        layer_sizes = zip(hidden_layers[:-1], hidden_layers[1:])
        self.hidden_layers.extend([nn.Linear(h1, h2) for h1, h2 in layer_sizes])
        # Ouput layer
        self.output = nn.Linear(hidden_layers[-1], output_size)
        
    def forward(self, x):
        ''' Forward pass through the network, returns the output logits.'''
        for each in self.hidden_layers:
            x = F.relu(each(x))
            # x = self.dropout(x)
        x = self.output(x)
        return x
    

##################################################################################################
##################################################################################################


def PredictController(model, state, scalerX=scalerX):
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


##################################################################################################
##################################################################################################


def model_birth(identifier=identifier, structure=structure):
    """Creates a network from a saved model.state_dict with the identifier `identifier`
    in `..\Models\` with the structure stored in `structure`."""

    # network structure parameters:
    input_size = structure['input_size']
    output_size = structure['output_size']
    hidden_layers = structure['hidden_sizes']

    # network initialization:
    model = NetworkReduced(input_size=input_size, output_size=output_size, hidden_layers=hidden_layers)

    # network loading
    path = models_path + root_name_given_to_state_dict + identifier
    model.load_state_dict(torch.load(path))

    # setting the network in evaluation mode
    # (setting dropout and batch normalization layers to evaluation mode)
    model.eval()

    return model


##################################################################################################
##################################################################################################


def PredictController(model, state, scalerX=scalerX):
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


##################################################################################################
##################################################################################################


# To call 'model' from Matlab:
model = model_birth(identifier=identifier)

# To call 'K' from Matlab:
if identifier[0:2] != '5D':
    state_ref = [20., 2000.]
else:
    state_ref = [20., 2000., 2.5, 0.33, 0.1568]

K = PredictController(model, state=state_ref)

