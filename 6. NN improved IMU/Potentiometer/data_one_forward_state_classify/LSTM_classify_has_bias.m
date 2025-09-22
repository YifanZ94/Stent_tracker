clear; clc
load('DataSet_train_slow.mat')
%%
% inputs: X(acc,v,delta_disp), state_By_dynamics, time, gyro_amp, acc_amp
% target: displacement, velocity, acc ref

T_thres = 2e-4;
for n = 1:size(InputCell,1)
    T = TargetCell{n};
    T_state = 0;
    for j = 1:size(T,2)
        if abs(T(j)) < T_thres
            T_state(j) = 0;
        else
            T_state(j) = 1;
        end
    end
    TargetCell{n,2} = categorical(T_state);
end

training_dim = [1,2,3,6,7];
targer_dim = 2;
idxTrain = size(InputCell,1);
train_inputs_set = InputCell(1:idxTrain);
train_target_set = TargetCell(1:idxTrain,2);
% test_inputs_set = InputCell(idxTrain+1:end);
% test_target_set = TargetCell(idxTrain+1:end,2);

%% prepare data for traning
n_training = size(train_inputs_set,1);
% n_test = size(test_inputs_set,1);

T_thres = 2e-4;
for n = 1:n_training
    X = train_inputs_set{n};
    XTrain{n,1} = X(training_dim,:);
    TTrain{n,1} = TargetCell{n,2};
end

%% normalize by mean and var
% X = cat(2,XTrain{:});
% muX = mean(X,2);
% muT = mean(T,2);

% muX = zeros(size(training_dim,2),1);
% muT = 0;
% sigmaX = std(X,0,2);
% sigmaT = 1;
% 
% for n = 1:numel(XTrain)
% %     XTrain{n,1} = (XTrain{n} - muX) ./ sigmaX;
% %     TTrain{n,1} = (TTrain{n} - muT) ./ sigmaT;
%     XTrain{n,1} = XTrain{n} ./ sigmaX;
%     TTrain{n,1} = TTrain{n} ./ sigmaT;
% end
 
%%  Define the NN architecture
layers = [
    sequenceInputLayer(numel(training_dim))
%     lstmLayer(100,'OutputMode','sequence')
    gruLayer(50,'OutputMode','sequence')

%     imageInputLayer([7 28 1])
%     convolution2dLayer([3 5], 4) 
%     reluLayer
%     maxPooling2dLayer(3,'Stride',3)
          
    fullyConnectedLayer(60)
    dropoutLayer(0.5)
    fullyConnectedLayer(40)
    dropoutLayer(0.5)
    fullyConnectedLayer(2)
    softmaxLayer("Name","softmax")
    classificationLayer("Name","classification")];

miniBatchSize = 10;
options = trainingOptions("adam", ...
    MaxEpochs= 200, ...
    MiniBatchSize= miniBatchSize, ...
    SequencePaddingDirection="left", ...
    InitialLearnRate= 0.02, ...
    GradientThreshold=1, ...
    Shuffle="every-epoch", ...
    Plots="training-progress", ...
    Verbose=0);

% ExecutionEnvironment = 'gpu'
%% Train
net = trainNetwork(XTrain,TTrain,layers,options);
save('Trained_NN.mat', 'net' )

% YTest = predict(net,XTest,SequencePaddingDirection="left");

%% Test for open loop (Have the real data as target) prediction for future
% 1:n_test

% for test_group_index = 1:n_test
% cat_test_input = test_inputs_set{test_group_index};
% cat_test_target = test_target_set{test_group_index};
% 
% XTest = cat_test_input(training_dim,:) ;
% TTest = cat_test_target ;
% 
% YPred{test_group_index} = classify(net,XTest,'MiniBatchSize',miniBatchSize);
% acc_pred(test_group_index) = mean(YPred{test_group_index} == TTest);
% acc_dynamics(test_group_index) = mean(categorical(cat_test_input(4,:)) ~= TTest);
% 
% end

disp('done')

% 'muT','muX','sigmaT','sigmaX'
%%
% plot(acc_pred)
% hold on
% plot(acc_dynamics)
% legend('prediction', 'dynamics')
% ylabel('Accuracy')
% xlabel('Test group')

%%
% figure
% subplot(411)
% n = 2;
% plot(TargetCell{n,1})
% hold on
% plot([1 size(TargetCell{n,1},2)], [T_thres  T_thres])
% ylabel('\DeltaS')
% subplot(412)
% plot(TargetCell{n,2})
% ylabel('ref')
% subplot(413)
% plot(YPred{1,n})
% ylabel('pred')
% subplot(414)
% plot(categorical(~InputCell{n}(4,:)))
% ylabel('dynamic')
% xlabel('sample')

%% Invert normalization to the actual scale
function Y = denormalize(X,sigma,mu)
    Y = X*sigma + mu;
end

% function Y = denormalize(X, maxX, minX)
%     Y = X.*(maxX-minX) + minX;
% end