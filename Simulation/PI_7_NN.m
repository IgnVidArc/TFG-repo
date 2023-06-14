%% LOADING NEURAL NETWORK FROM FILE
model_name = '../Models/Model_002_try_list.onnx';
% % net = importNetworkFromPyTorch(model_name)
net = importNetworkFromPyTorch(model_name)
% net = importONNXNetwork(model_name)

%% CREATING IT FROM SCRATCH:

numInputs = 1;              % Número de entradas
numLayers = 2;              % Número de capas
biasConect = [1; 1];        % Vector booleano de size(numLayers, 1)
inputConnect = [1; 0];      % Matriz booleana de size(numLayers, numInputs)
outputConnect = [0, 1];     % Vector booleano de size(1, numLayers)
layerConnect = zeros(numLayers);
for row = 2:numLayers       % Matriz booleana de size(numLayers, numLayers)
    layerConnect(row, row-1) = row-1;
end

net = network(numInputs, numLayers, biasConect, inputConnect, layerConnect, outputConnect);

% net.inputs{1}
% net.layers{1}, net.layers{2}
% net.biases{1}
% net.inputWeights{1,1}, net.layerWeights{2,1}
% net.outputs{2}

% net.inputs{1}.range = [10 30; 100 3100];
% plot(net)

net.plotFcns


%% TUTORIAL:
% https://www.youtube.com/watch?v=5soi-eOUNsc
% x = 0:0.01:10;
% y = x.^3;
% 
% net = newff(minmax(x), [20,1], {'logsig', 'purelin', 'trainlm'});
% net.trainParam.epochs = 4000;
% net.trainParam.goal = 1e-25;
% net.trainParam.lr = 0.01;
% net = train(net,x,y);
