# %% [markdown]
# https://medium.com/mlearning-ai/create-a-neural-network-with-pytorch-lightning-in-just-100-lines-of-code-43eccbf3fba

# %% [markdown]
# ## **NN_CONTROLLER**
# 
# 

# %%
# PACKAGES

# data handling
import numpy as np
from sklearn.preprocessing import StandardScaler
from sklearn.model_selection import train_test_split
from sklearn.metrics import r2_score, mean_squared_error

# deep learning
import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.utils.data import Dataset, DataLoader, random_split
import pytorch_lightning as pl
from pytorch_lightning.callbacks.early_stopping import EarlyStopping

# utilities:
import copy
import matplotlib.pyplot as plt
import pandas as pd
from joblib import dump

# %% 
class GetDataset(Dataset):
    "Geting features and outputs ndarray from file."
    def __init__(self, filename, ninputs, noutputs, delimiter: str=','):
        # data loading
        xy = np.loadtxt(filename, delimiter=delimiter, dtype=np.float32)
        self.X = xy[:, :ninputs]
        self.y = xy[:, ninputs:ninputs+noutputs]
        self.n_samples = xy.shape[0]
    
    def __len__(self):
        return self.n_samples
    
    def __getitem__(self, idx):
        return self.X[idx], self.y[idx]
    
    
class DatasetFromXy(Dataset):
    def __init__(self, X, y):
        self.X = torch.from_numpy(X)
        self.y = torch.from_numpy(y)
        self.n_samples = X.shape[0]

    def __len__(self):
        return self.n_samples
    
    def __getitem__(self, idx):
        return self.X[idx], self.y[idx]

# %% DATASET GENERATION
filename = '../Datasets/dataset_005.csv'
ninputs = 2
noutputs = 10

data_raw = GetDataset(filename=filename, ninputs=ninputs, noutputs=noutputs)
X_raw = data_raw.X
y_raw = data_raw.y

scalerX = StandardScaler()
X_scaled = scalerX.fit_transform(X_raw)

data_scaled = DatasetFromXy(X_scaled, y_raw)

test_set, train_set = random_split(data_scaled, lengths=[.2,.8], generator=torch.Generator())

print(f'Number of training samples: {len(train_set):4}\nNumber of testing samples: {len(test_set):5}')
nsamples = len(test_set)

# %% DATALOADERS AND TRAIN/TEST BATCHES 
train_batch_size = 100
test_batch_size = 20

train_loader = DataLoader(train_set, batch_size=train_batch_size, shuffle=True, num_workers=0)
test_loader = DataLoader(test_set, batch_size=test_batch_size, shuffle=False, num_workers=0)

# %% [markdown]
# ## Network

# %%
# NEURAL NETWORK DEFINITION
class Network(pl.LightningModule):
    def __init__(self, input_size, output_size, hidden_layers, learning_rate, drop_p=0.2):
        ''' Builds a fully connected network with arbitrary hidden layers.
        
            Arguments
            ---------
            input_size: integer, size of the input layer.
            output_size: integer, size of the output layer.
            hidden_layers: list of integers, the sizes of the hidden layers.
            learning_rate: learning rate for the optimizer.
            drop_p: float = 0.2, drop out probability.
        '''
        super().__init__()

        self.input_size = input_size
        self.output_size = output_size
        self.hidden_sizes = hidden_layers
        self.structure_dict = {'input_size': input_size,
                               'output_size': output_size,
                               'hidden_sizes': hidden_layers}

        # Input to a hidden layer
        self.hidden_layers = nn.ModuleList([nn.Linear(input_size, hidden_layers[0])])
        
        # Add a variable number of more hidden layers
        layer_sizes = zip(hidden_layers[:-1], hidden_layers[1:])
        self.hidden_layers.extend([nn.Linear(h1, h2) for h1, h2 in layer_sizes])
        
        # Ouput layer
        self.output = nn.Linear(hidden_layers[-1], output_size)
        
        # Dropout probabilty
        self.dropout = nn.Dropout(p=drop_p)

        # Loss function
        self.loss_fun = nn.MSELoss()    # Mean Squared Error Loss

        # Optimizer learning rate
        self.learning_rate = learning_rate

        # optimizer:
        self.optimizer = []

        # extra: monitoring training loss
        # self.training_loss = []
        # self.test_batch_idx = []
        

    def forward(self, x):
        ''' Forward pass through the network, returns the output logits.'''
        for each in self.hidden_layers:
            x = F.relu(each(x))
            x = self.dropout(x)
        x = self.output(x)
        return x
    

    def configure_optimizers(self):
        optimizer = torch.optim.Adam(self.parameters(), lr=self.learning_rate)
        self.optimizer = optimizer
        return optimizer
    

    def training_step(self, train_batch, batch_idx):
        "Definition of the training loop."
        X, y = train_batch                  # (extracting features and outputs)
        # y = y.type(torch.float32)         # (just in case)
        # forward pass
        y_pred = self.forward(X).squeeze()  # 
        # compute loss
        loss = self.loss_fun(y_pred, y)     # 
        self.log_dict({'train_loss': loss}, on_step=False, on_epoch=True, prog_bar=True,
                      logger=True, enable_graph=True)
        
        # extra: monitoring training loss:
        # self.training_loss.append(np.mean(loss.item()))
        # self.test_batch_idx.append(batch_idx)
        # extra:
        # writer.add_scalar('Train Loss', loss)
        return loss
    

    def test_step(self, test_batch, batch_idx):
        X, y = test_batch
        # forward pass
        y_pred = self.forward(X).squeeze()        
        # compute metrics
        loss = self.loss_fun(y_pred, y)
        r2 = r2_score(y_pred, y)
        rmse = np.sqrt(mean_squared_error(y_pred, y))
        self.log_dict({'Cost function': loss, 'r2': r2, 'Root Mean Square Error': rmse},
                      on_step=False, on_epoch=True, prog_bar=True, logger=True)
        return loss

# %% [markdown]
# ### Network definition

# %%
# NETWORK DEFINITION

# Number of layers and their sizes:
input_size = ninputs
output_size = noutputs
hidden_layers = [20, 40, 40, 20] # [32, 64, 64, 32]

# Other hyperparameters:
max_epochs = 500
lr = 0.0001 * 10
drop_p = 0.1

model = Network(input_size=input_size, output_size=output_size, hidden_layers=hidden_layers,
                learning_rate=lr, drop_p=drop_p)

# Including early stoping: `patience` is the key parameter here.
patience = 20
early_stop_callback = EarlyStopping(monitor="train_loss", min_delta=0.00, patience=patience, verbose=True, mode="min")

# %% [markdown]
# ### Training

# %%
# DEFINITION OF THE TRAINER
trainer = pl.Trainer(accelerator='cpu', devices='auto', max_epochs=max_epochs,
                     callbacks=[early_stop_callback], log_every_n_steps=8)

# TRAINING
trainer.fit(model=model, train_dataloaders=train_loader)
# writer.flush()
# writer.close()


# %% [markdown]
# ### Testing

# %%
# TEST OUTPUT
trainer.test(model=model, dataloaders=test_loader)

# %% [markdown]
# ### Saving & Loading Model to Disk

# %%
# ----- IDENTIFIER ------
identifier = '005_20_40_40_20'
# ------ -------- -------

# SAVING THE STATE DICT
state_dict_path = '../Models/' + 'state_dict_' + identifier
torch.save(model.state_dict(), state_dict_path)

# SAVING THE STRUCTURE DICT
structure_dict = model.structure_dict
dump(structure_dict, '../Models/' + 'structure_' + identifier, compress=True)

# SAVING THE STANDARD SCALER
dump(scalerX, '../Models/scalerX_' + identifier[0:3] + '.bin', compress=True)

# %% [markdown]
# 

# %% [markdown]
# 

# %% [markdown]
# https://pytorch.org/tutorials/beginner/saving_loading_models.html

# %%
# Print model's state_dict
print("Model's state_dict:")
for param_tensor in model.state_dict():
    print(param_tensor, "\t", model.state_dict()[param_tensor].size())

# Print optimizer's state_dict
print("\nOptimizer's state_dict:")
for var_name in model.optimizer.state_dict():
    print(var_name, "\t", model.optimizer.state_dict()[var_name])

