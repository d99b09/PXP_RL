import numpy as np

from PXP_env import *

from keras.models import Sequential
from keras.layers import Dense, Activation, Flatten
from keras.optimizers import Adam

from rl.agents.dqn import DQNAgent
from rl.policy import BoltzmannQPolicy
from rl.memory import SequentialMemory

env = PXP_env()
env.configurate()

nb_actions = 4

model = Sequential()

model.add(Flatten(input_shape=(1,) + (17,)))
model.add(Dense(32))
model.add(Activation('relu'))
model.add(Dense(32))
model.add(Activation('relu'))
model.add(Dense(32))
model.add(Activation('relu'))
model.add(Dense(nb_actions))
model.add(Activation('linear'))

print(model.summary())

memory = SequentialMemory(limit=100000, window_length=1)
policy = BoltzmannQPolicy()
dqn = DQNAgent(model=model, nb_actions=nb_actions, memory=memory, nb_steps_warmup=10,
               target_model_update=1e-2, policy=policy)
dqn.compile(Adam(lr=1e-3), metrics=['mae'])

dqn.fit(env, nb_steps=1000000, visualize=True, verbose=2)

dqn.save_weights('dqn_{}_weights.h5f'.format('PXP'), overwrite=True)

dqn.test(env, nb_episodes=5, visualize=True)