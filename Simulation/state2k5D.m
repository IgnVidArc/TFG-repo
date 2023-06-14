function K = state2k(state)
    
    pyfile = '../Networks/state2k.py';
    model = pyrunfile(pyfile, 'model');
    K = double(pyrunfile(pyfile, 'K', model=model, state=state));
end