import pytorch as torch 
import pytorch.nn as nn
import numpy 
import matplotlib.pyplot as plt 
import torch.optim as optim

from sklearn.preprocessing import StandardScaler, MinMaxScaler, RobustScaler, QuantileTransformer

class Network(nn.Module):
    def __init__(self, netParams, trainParams):
        super(Network,self).__init__()
        self.state_n = netParams['state_n']
        self.u_n = netParams['action_n']
        self.in_n = self.state_n + self.u_n
        self.out_n = netParams['output_n']
        self.prob = netParams['prob'] #denotes whether or not this is a PNN
        self.actor_var = netParams['sigma']
        self.hidden_w = nParams['hidden']
        self.depth = netParams['depth']
        self.activation = netParams['activation']
        self.d = netParams['dropout']
        self.discrete = netParam['discrete']

        if self.prob:
            self.out_n *= 2

        self.lr = trainParams['lr']
        self.pre = netParams['preprocess']
        self.post = netParams['postprocess']
        self.epochs = netParams['epochs']
        loss = netParams['loss_fnc']
        if loss == "policy_gradient":
            self.loss_fnc = None
        elif loss == "MSE":
            self.loss_fnc = nn.MSELoss()
        else:
            assert False

        self.scalarInput = StandardScaler() #or any of the other scalers...look into them 
        self.scalarOutput = StandardScaler()
                
        self.optimizer =  optim.Adam(super(Network, self).parameters(), lr=self.lr)

        layers = []
        layers.append(('dynm_input_lin', nn.Linear(
                self.in_n, self.hidden_w)))       # input layer
        layers.append(('dynm_input_act', self.activation))
        layers.append(('dynm_input_dropout', nn.Dropout(p = self.d)))
        for d in range(self.depth):
            layers.append(('dynm_lin_'+str(d), nn.Linear(self.hidden_w, self.hidden_w)))
            layers.append(('dynm_act_'+str(d), self.activation))
            layers.append(('dynm_dropout_' + str(d), nn.Dropout(p = self.d)))


        layers.append(('dynm_out_lin', nn.Linear(self.hidden_w, self.out_n)))
        self.features = nn.Sequential(OrderedDict([*layers]))

    
    def preProcess(self, inputs, outputs) {
        #normalize the input vectors
        self.scalarInput.fit(inputs)
        norm = self.scalarInput.transform(inputs)
        self.scalarOutput.fit(outputs)
        normOut = self.scalarOutput.transform(outputs)
        return norm, normOut
    }

    def postProcess(self, outputs) {
        #depeneds what we are trying to do 
        return outputs
    }
    def forward(self, inputs){
        x = self.features(input)
        x = self.postProcess(x)
        return x 
    }

    def predict(self, input){
        input = self.preProcess(input)
        self.eval()
        x = self.features(input)
        prediction = self.postProcess(x)
        return prediction

    }

    def train(self, inputs, outputs, advantages = None) {
        #experience replay is kept in the agent.py code 
        #advantage function: we need to only estimate the value function. This is directl put in 
        #policy gradient: we pass in a SEQUENCE of state action pairs.
            #we pass in our respective state to the network to output a mean and variance
            #we also have actions that were taken beforehand with respective advantages
        #inputs: always sequence of states
        #outputs: can be state or action space depending on critic or actor network 
        #advantage: always advantage values or none depending if actor or critic
        self.train()
        for i in range(self.epochs):
            normIn, normOut = self.preProcess(inputs ,outputs)
            out = self.forward(normIn)
            if self.loss_fnc != None: #MSELoss
                assert advantages == None 
                loss = self.loss_fnc(out, outputs)
                loss.backward()
                self.optimizer.step()
            else: #policy gradient
                #each row is a sample. Outputs represent our actions!
                #we compute the log probabilities that we would execute those actions given our current policy
                means = out[: int(self.out_n/2)]
                std = out[int(self.out_n/2)p
                prob = np.exp(-(1/2)*np.divide(np.square(means - outputs), std))
                prob = 1/((2*np.pi)**(means.shape[0]) *np.prod(std)) ** (1/2)
                gradient = -np.sum(prob*advantages)
                gradient.backward()
                self.optimizer.step()
    }
