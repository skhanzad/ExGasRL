import numpy as np
from demo import LunarLandingModel
import torch

# Extend LunarLandingModel for XAI if needed
class XAILunarLandingModel(LunarLandingModel):
    def load_weights(self, path):
        self.load_state_dict(torch.load(path, map_location=torch.device('cpu')))
        self.eval()
    def predict(self, state):
        with torch.no_grad():
            x = torch.tensor(state, dtype=torch.float32)
            q = self.forward(x)
            return torch.argmax(q).item()
    def q_values(self, state):
        with torch.no_grad():
            x = torch.tensor(state, dtype=torch.float32)
            q = self.forward(x)
            return q.numpy()

model = XAILunarLandingModel(input_size=8, hidden_size=64, output_size=4)
model.load_weights("path/to/weights.pt")  # TODO: update path

def predict(state):
    state = np.array(state, dtype=np.float32)
    action = int(model.predict(state))
    q_values = model.q_values(state).tolist()
    return action, q_values 