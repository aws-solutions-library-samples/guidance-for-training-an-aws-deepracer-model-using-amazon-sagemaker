# How to Change RL Agent Algorithm in Amazon SageMaker notebook for AWS DeepRacer


## **SageMaker notebook setup**


Model sync behavior is coded in src/markov/training_worker.py 

```
if graph_manager.agent_params.algorithm.distributed_coach_synchronization_type == 
            DistributedCoachSynchronizationType.SYNC:
    graph_manager.save_checkpoint()
else:
    graph_manager.occasionally_save_checkpoint()
```


To enable sync models to the s3 bucket for multiple rollout, set the parameter in src/markov/presets/preset.py

```
agent_params.algorithm.distributed_coach_synchronization_type = DistributedCoachSynchronizationType.SYNC
```


To change the algorithm to another RL agent in the RL Coach library, we need to make changes in several file related to algorithm selection, hyperparameters, graph manager, and neural network output layers.

**Step 1: Replace the preset file**

First we need to edit the agent specific parameters in src/markov/presets/preset.py. 

For example, for PPO

```
agent_params = ClippedPPOAgentParameters()
```

or for DQN

```
agent_params = DQNAgentParameters()
```

To set new values for the agent hyperparameter, see the documentation for the algorithm and set them accordingly. For example “agent_params.algorithm.gae_lambda” is a parameter specific to PPO and will throw an error if used for a DQN agent.


preset.py example for DQN:

```
from rl_coach.architectures.layers import Conv2d, Dense, BatchnormActivationDropout
from rl_coach.architectures.middleware_parameters import FCMiddlewareParameters
from rl_coach.architectures.embedder_parameters import InputEmbedderParameters
from rl_coach.agents.dqn_agent import DQNAgentParameters
from rl_coach.base_parameters import VisualizationParameters, PresetValidationParameters
from rl_coach.core_types import TrainingSteps, EnvironmentEpisodes, EnvironmentSteps
from rl_coach.environments.gym_environment import GymVectorEnvironment
from rl_coach.exploration_policies.categorical import CategoricalParameters
from rl_coach.filters.filter import InputFilter
from rl_coach.filters.observation.observation_rgb_to_y_filter import ObservationRGBToYFilter
from rl_coach.filters.observation.observation_stacking_filter import ObservationStackingFilter
from rl_coach.filters.observation.observation_to_uint8_filter import ObservationToUInt8Filter
from rl_coach.graph_managers.basic_rl_graph_manager import BasicRLGraphManager
from rl_coach.graph_managers.graph_manager import ScheduleParameters
from rl_coach.schedules import LinearSchedule

from rl_coach.base_parameters import DistributedCoachSynchronizationType
####################
# Graph Scheduling #
####################

schedule_params = ScheduleParameters()
schedule_params.improve_steps = TrainingSteps(10000000)
schedule_params.steps_between_evaluation_periods = EnvironmentEpisodes(40)
schedule_params.evaluation_steps = EnvironmentEpisodes(5)
schedule_params.heatup_steps = EnvironmentSteps(0)

#########
# Agent #
#########
agent_params = DQNAgentParameters()

agent_params.network_wrappers['main'].learning_rate = 0.0003

agent_params.network_wrappers['main'].input_embedders_parameters['observation'].activation_function = 'relu'
agent_params.network_wrappers['main'].middleware_parameters.activation_function = 'relu'

agent_params.network_wrappers['main'].batch_size = 64
agent_params.network_wrappers['main'].optimizer_epsilon = 1e-5
agent_params.network_wrappers['main'].adam_optimizer_beta2 = 0.999

agent_params.algorithm.discount = 0.999
agent_params.algorithm.num_steps_between_copying_online_weights_to_target = EnvironmentEpisodes(20)
agent_params.algorithm.num_consecutive_playing_steps = EnvironmentEpisodes(20)

agent_params.algorithm.distributed_coach_synchronization_type = DistributedCoachSynchronizationType.SYNC


###############
# Environment #
###############
SilverstoneInputFilter = InputFilter(is_a_reference_filter=True)

SilverstoneInputFilter.add_observation_filter('observation', 'to_grayscale', ObservationRGBToYFilter())
SilverstoneInputFilter.add_observation_filter('observation', 'to_uint8', ObservationToUInt8Filter(0, 255))
SilverstoneInputFilter.add_observation_filter('observation', 'stacking', ObservationStackingFilter(1))

env_params = GymVectorEnvironment()
env_params.default_input_filter = SilverstoneInputFilter
env_params.level = 'DeepRacerRacetrackCustomActionSpaceEnv-v0'

vis_params = VisualizationParameters()
vis_params.dump_mp4 = False

########
# Test #
########
preset_validation_params = PresetValidationParameters()
preset_validation_params.test = True
preset_validation_params.min_reward_threshold = 400
preset_validation_params.max_episodes_to_achieve_reward = 1000

graph_manager = BasicRLGraphManager(agent_params=agent_params, env_params=env_params,
                                    schedule_params=schedule_params, vis_params=vis_params,
                                    preset_validation_params=preset_validation_params)
```


**Step 2: Update the network model save head**

We need to update the network head we need to save to the model store. This will be done first locally in “src/markov/utils.py” via 


```
def write_frozen_graph(graph_manager):
    if not os.path.exists(SM_MODEL_OUTPUT_DIR):
        os.makedirs(SM_MODEL_OUTPUT_DIR)
    
    output_head = ['main_level/agent/main/online/network_0/q_values_head_0/q_values_head_0_target']
    frozen = tf.graph_util.convert_variables_to_constants(graph_manager.sess, graph_manager.sess.graph_def, output_head)
    tf.train.write_graph(frozen, SM_MODEL_OUTPUT_DIR, 'model.pb', as_text=False)
```


Then, written to the s3 bucket by “src/markov/s3_boto_data_store.py”


```
# Upload the frozen graph which is used for deployment
if self.graph_manager:
    utils.write_frozen_graph(self.graph_manager)
    # upload the model_<ID>.pb to S3. NOTE: there's no cleanup as we don't know the best checkpoint
    iteration_id = self.graph_manager.level_managers[0].agents['agent'].training_iteration
    frozen_graph_fpath = utils.SM_MODEL_OUTPUT_DIR + "/model.pb"
    frozen_graph_s3_name = "model_%s.pb" % iteration_id
    s3_client.upload_file(Filename=frozen_graph_fpath,
                      Bucket=self.params.bucket,
                      Key=self._get_s3_key(frozen_graph_s3_name))
    logger.info("saved intermediate frozen graph: {}".format(self._get_s3_key(frozen_graph_s3_name)))
```


Step 3: Do we need this one?

Finally, we need to change the Sagemaker graph manager “src/markov/sagemaker_graph_manager.py” with the agent parameters. The Sagemaker graph manager runs when the default files are missing.


```
from rl_coach.agents.clipped_ppo_agent import ClippedPPOAgentParameters
from rl_coach.agents.dqn_agent import DQNAgentParameters
from rl_coach.base_parameters import VisualizationParameters, PresetValidationParameters, DistributedCoachSynchronizationType
from rl_coach.core_types import TrainingSteps, EnvironmentEpisodes, EnvironmentSteps
from rl_coach.environments.gym_environment import GymVectorEnvironment
from rl_coach.exploration_policies.categorical import CategoricalParameters
from rl_coach.exploration_policies.e_greedy import EGreedyParameters
from rl_coach.filters.filter import InputFilter
from rl_coach.filters.observation.observation_rgb_to_y_filter import ObservationRGBToYFilter
from rl_coach.filters.observation.observation_stacking_filter import ObservationStackingFilter
from rl_coach.filters.observation.observation_to_uint8_filter import ObservationToUInt8Filter
from rl_coach.graph_managers.basic_rl_graph_manager import BasicRLGraphManager
from rl_coach.graph_managers.graph_manager import ScheduleParameters
from rl_coach.schedules import LinearSchedule

import json


def get_graph_manager(**hp_dict):
    ####################
    # All Default Parameters #
    ####################
    params = {}
    params["batch_size"] = int(hp_dict.get("batch_size", 64))
    params["num_epochs"] = int(hp_dict.get("num_epochs", 10))
    params["stack_size"] = int(hp_dict.get("stack_size", 1))
    params["lr"] = float(hp_dict.get("lr", 0.0003))
    params["exploration_type"] = (hp_dict.get("exploration_type", "egreedy")).lower()
    params["e_greedy_value"] = float(hp_dict.get("e_greedy_value", .05))
    params["epsilon_steps"] = int(hp_dict.get("epsilon_steps", 10000))
    params["beta_entropy"] = float(hp_dict.get("beta_entropy", .01))
    params["discount_factor"] = float(hp_dict.get("discount_factor", .999))
    params["loss_type"] = hp_dict.get("loss_type", "Mean squared error").lower()
    params["num_episodes_between_training"] = int(hp_dict.get("num_episodes_between_training", 20))
    params["term_cond_max_episodes"] = int(hp_dict.get("term_cond_max_episodes", 100000))
    params["term_cond_avg_score"] = float(hp_dict.get("term_cond_avg_score", 100000))

    params_json = json.dumps(params, indent=2, sort_keys=True)
    print("Using the following hyper-parameters", params_json, sep='\n')

    ####################
    # Graph Scheduling #
    ####################
    schedule_params = ScheduleParameters()
    schedule_params.improve_steps = TrainingSteps(params["term_cond_max_episodes"])
    schedule_params.steps_between_evaluation_periods = EnvironmentEpisodes(40)
    schedule_params.evaluation_steps = EnvironmentEpisodes(5)
    schedule_params.heatup_steps = EnvironmentSteps(0)

    #########
    # Agent #
    #########
    #agent_params = ClippedPPOAgentParameters()
    agent_params = DQNAgentParameters()


    agent_params.network_wrappers['main'].learning_rate = params["lr"]
    
    #agent_params.network_wrappers['main'].input_embedders_parameters['observation'].activation_function = 'relu'
    agent_params.network_wrappers['main'].input_embedders_parameters = {
            'observation': InputEmbedderParameters(
                scheme=[
                    Conv2d(32, 8, 4),
                    BatchnormActivationDropout(batchnorm=True, activation_function='relu'),
                    Conv2d(32, 4, 2),
                    BatchnormActivationDropout(batchnorm=True, activation_function='relu'),
                    Conv2d(64, 4, 2),
                    BatchnormActivationDropout(batchnorm=True, activation_function='relu'),
                    Conv2d(64, 3, 1),
                    BatchnormActivationDropout(batchnorm=True, activation_function='relu'),
                    Dense(512),
                    BatchnormActivationDropout(activation_function='relu', dropout_rate=0.5),
                    Dense(512),
                    BatchnormActivationDropout(activation_function='relu', dropout_rate=0.5)
                ],
                activation_function='none')
            }


    #agent_params.network_wrappers['main'].middleware_parameters.activation_function = 'relu'
    agent_params.network_wrappers['main'].middleware_parameters = \
        FCMiddlewareParameters(
            scheme=[
                Dense(256),
                BatchnormActivationDropout(activation_function='relu', dropout_rate=0.5),
                Dense(128),
                BatchnormActivationDropout(activation_function='relu', dropout_rate=0.5)
            ],
            activation_function='none'
        )
    
    agent_params.network_wrappers['main'].batch_size = params["batch_size"]
    agent_params.network_wrappers['main'].optimizer_epsilon = 1e-5
    agent_params.network_wrappers['main'].adam_optimizer_beta2 = 0.999

    if params["loss_type"] == "huber":
        agent_params.network_wrappers['main'].replace_mse_with_huber_loss = True

    #agent_params.algorithm.clip_likelihood_ratio_using_epsilon = 0.2
    #agent_params.algorithm.clipping_decay_schedule = LinearSchedule(1.0, 0, 1000000)
    #agent_params.algorithm.beta_entropy = params["beta_entropy"]
    #agent_params.algorithm.gae_lambda = 0.95
    agent_params.algorithm.discount = params["discount_factor"]
    #agent_params.algorithm.optimization_epochs = params["num_epochs"]
    #agent_params.algorithm.estimate_state_value_using_gae = True
    agent_params.algorithm.num_steps_between_copying_online_weights_to_target = EnvironmentEpisodes(
        params["num_episodes_between_training"])
    agent_params.algorithm.num_consecutive_playing_steps = EnvironmentEpisodes(params["num_episodes_between_training"])

    agent_params.algorithm.distributed_coach_synchronization_type = DistributedCoachSynchronizationType.SYNC

    if params["exploration_type"] == "categorical":
        agent_params.exploration = CategoricalParameters()
    else:
        agent_params.exploration = EGreedyParameters()
        agent_params.exploration.epsilon_schedule = LinearSchedule(1.0,
                                                                   params["e_greedy_value"],
                                                                   params["epsilon_steps"])

    ###############
    # Environment #
    ###############
    DeepRacerInputFilter = InputFilter(is_a_reference_filter=True)
    DeepRacerInputFilter.add_observation_filter('observation', 'to_grayscale', ObservationRGBToYFilter())
    DeepRacerInputFilter.add_observation_filter('observation', 'to_uint8', ObservationToUInt8Filter(0, 255))
    DeepRacerInputFilter.add_observation_filter('observation', 'stacking',
                                                  ObservationStackingFilter(params["stack_size"]))

    env_params = GymVectorEnvironment()
    env_params.default_input_filter = DeepRacerInputFilter
    env_params.level = 'DeepRacerRacetrackCustomActionSpaceEnv-v0'

    vis_params = VisualizationParameters()
    vis_params.dump_mp4 = False

    ########
    # Test #
    ########
    preset_validation_params = PresetValidationParameters()
    preset_validation_params.test = True
    preset_validation_params.min_reward_threshold = 400
    preset_validation_params.max_episodes_to_achieve_reward = 10000

    graph_manager = BasicRLGraphManager(agent_params=agent_params, env_params=env_params,
                                        schedule_params=schedule_params, vis_params=vis_params,
                                        preset_validation_params=preset_validation_params)
    return graph_manager, params_json
```






