

pyfile = '../Networks/state2k.py';
tic
name = 'state_dict_002_A';
model = pyrunfile(pyfile, 'model');
toc
%%
state=[14.92, 1492.];
K = single(pyrunfile(pyfile, 'K', model=model, state=state));

%%

state=[14.92, 1492.];
tic
K = state2k(state)
toc
%%
pyfile = '../Networks/state2k.py';
model = pyrunfile(pyfile, 'model');
%%
save('model_002_20_40_40_20', "model")