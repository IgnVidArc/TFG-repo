# PACKAGES

# data handling
import numpy as np
from sklearn.preprocessing import StandardScaler
# deep learning
import torch
import torch.nn as nn
import torch.nn.functional as F

# to load the loading the Standard Scaler
from joblib import load

# scalerX:
# path:
scaler_path = '../Models/'
# TO CHANGE: name of the one to use:
scaler_name = 'scalerX_002.bin'
# Loading reference scaler:
scalerX = load(scaler_path + scaler_name)

# import sys
# # sys.exit()

# REDUCED NEURAL NETWORK DEFINITION
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