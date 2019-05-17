# Lab 1: AWS DeepRacer の強化学習モデルを構築しましょう!

# Notes
We are continuously looking to improve the AWS DeepRacer service to provide a better customer experience. As such please always refer to the latest lab content in GitHub as prior content may be outdated. If you do have any technical questions please ask the workshop facilitators, and for those of you working through the lab at home, please post your questions to the [AWS DeepRacer forum](https://forums.aws.amazon.com/forum.jspa?forumID=318).


# Overview
The lab has four goals:

1. familiarize you with the AWS DeepRacer service in the AWS console,
2. explain the concepts needed to get you started training a model,
3. explain how you can compete in the DeepRacer League, both the Virtual Circuit and the Summit Circuit, and
4. explain how you can improve your model.

The lab is split into three sections:

1. Section 1: Training your first model,
2. Section 2: Competing in the AWS DeepRacer League, and
3. Section 3: Model training and improving your model.

Goals one and two are covered in Section 1, goal three is covered in Section 2, and goal four is covered in Section 3.


# Format
You will have 50 minutes to complete the lab and this is enough time to thoroughly go through the content, explore the service, and start training your first AWS DeepRacer reinforcement learning (RL) model. Section 1 should take about 25 to 35 minutes, Section 2 should take about 5 minutes, and Section 3 will take more time than you have in the workshop but is for use at home.

The lab will provide detail on the various components in the AWS DeepRacer service in the console and you will get the chance to try them all out. You should will start training your model at the end of Section 1.


# Hints
- Please make sure you save your reward function, and download your trained model from the burner account. You will lose access to the account after the Summit, and the account will be wiped.
- For those eager to start training a job, our hint would be take your time and familiarize yourself with the concepts first, before starting model training. 
- Please ask questions as you progress through the lab and feel free to have discussions at your table. 
- Lastly, when you do start a training job, run it for at least 90 minutes (on the re:Invent track). It takes 6 minutes to spin up the services needed and your model needs time to explore the track before it will manage to complete a lap.
- If you want to continue learning after the lab, please check out the new course by the AWS Training and Certification team, called [AWS DeepRacer: Driven by Reinforcement Learning](https://www.aws.training/learningobject/wbc?id=32143)

# Section 1: Training your first model
## Step 1: Login to the AWS DeepRacer service
Log into the [AWS Console](https://signin.aws.amazon.com/signin?redirect_uri=https%3A%2F%2Fconsole.aws.amazon.com%2Fconsole%2Fhome%3Fnc2%3Dh_ct%26src%3Dheader-signin%26state%3DhashArgs%2523%26isauthcode%3Dtrue&client_id=arn%3Aaws%3Aiam%3A%3A015428540659%3Auser%2Fhomepage&forceMobileApp=0) using the account details provided.

Make sure you are in the **North Virginia** region and navigate to [AWS DeepRacer](https://console.aws.amazon.com/deepracer/home?region=us-east-1) (https://console.aws.amazon.com/deepracer/home?region=us-east-1).

From the AWS DeepRacer landing page, expand the pane on the left and select **Reinforcement learning**.

## Step 2: Model List Page
Once you select Reinforcement learning, you will land on the models page. This page shows a list of all the models you have created and the status of each model. If you want to create models, this is where you start the process. Similarly, from this page you can download, clone, and delete models.

![Model List Page](img/model_list_deepracer.png)

If you don't have any models this list will be empty, and you can create a model by choosing **Create model**.
Once you have created a model you can use this page to view the status of the model, for example is it training, or ready. A model status of "ready" indicates model training has completed and you can then download it, evaluate it, or submit it to a virtual race. You can click on the model's name to proceed to the **Model details** page. 

To create your first model select **Create model**.


## Step 3: Create model
This page gives you the ability to create an RL model for AWS DeepRacer and start training the model. There are a few sections on the page, but before we get to each please scroll all the way down the page and then all the way back up so you get a sense of what is to come. We are going to create a model that can be used by the AWS DeepRacer car to autonomously drive (take action) around a race track. We need to select the specific race track, provide the actions that our model can choose from, provide a reward function that will be used to incentivize our desired driving behavior, and configure the hyperparameters used during training. 


### <font color=blue>**Info**</font> **Buttons**
Throughout the console you will see <font color=blue>**Info**</font> buttons. When selected, an information pane will slide onto the screen from the right of the window. Info buttons will not navigate away from the current page, unless you select a link in the information pane. You can close the panes once you are done.


## 3.1 Model details
You should start at the top with Model Details. Here you can name your model and provide a description for your model. If this is the first time you use the service you should select the **Create Resources** button. This will create the IAM roles that AWS DeepRacer needs to call other AWS services on your behalf, the VPC stack used during training and evaluation, the AWS DeepRacer lambda function used to validate your Python 3 reward function, and the AWS DeepRacer S3 bucket where model artifacts will be stored. If you see an error in this section please let us know.


![Model Details](img/model_details.png)

Please enter a name and description for your model and scroll to the next section.


## 3.2 Environment simulation
As detailed in the workshop, training our RL model takes place on a simulated race track in our simulator, and in this section you will choose the track on which you will train your model. AWS RoboMaker is used to spin up the simulation environment.

When training a model, keep the track on which you want to race in mind. Train on the track most similar to the final track you intend to race on. While this isn't required and doesn't guarantee a good model, it will maximize the odds that your model will get its best performance on the race track. Furthermore, if you train on a straight track, don't expect your model to learn how to turn.

We will provide more details on the AWS DeepRacer League in Section 2, but here are things to keep in mind when selecting a track to train on if you intent to race in the League.

- For the [Summit Circuit](https://aws.amazon.com/deepracer/summit-circuit/), the live race track will be the re:Invent 2018 track, so train your model on the re:Invent track if you intend to race at any of the selected AWS Summits. 
- Each race in the Virtual Circuit will have its own new competition track and it won't be possible to directly train on the competition tracks. Instead we will make a track available that will be similar in theme and design to each competition track, but not identical. This ensures that models have to generalize, and can't just be over fitted to the competition track. 

For today's lab we want to get you ready to race at the Summit, time permitting, so please select the re:Invent 2018 track and scroll to the next section.


## 3.3 Action space
In this section you get to configure the action space that your model will select from during training, and also once the model has been trained. An action is a combination of speed and steering angle. In AWS DeepRacer we are using a discrete action space as opposed to a continuous action space. To build this discrete action space you will specify the maximum speed, the speed granularity, the maximum steering angle, and the steering granularity.

![action space](img/Action_Space.png)

Inputs

- Maximum steering angle is the maximum angle in degrees that the front wheels of the car can turn, to the left and to the right. There is a limit as to how far the wheels can turn and so the maximum turning angle is 30 degrees.
- Steering levels refers to the number of steering intervals between the maximum steering angle on either side.  Thus if your maximum steering angle is 30 degrees, then +30 degrees is to the left and -30 degrees is to the right. With a steering granularity of 5, the following steering angles, from left to right, will be in the action space: 30 degrees, 15 degrees, 0 degrees, -15 degrees, and -30 degrees. Steering angles are always symmetrical around 0 degrees.
- Maximum speeds refers to the maximum speed the car will drive in the simulator as measured in meters per second. 
- Speed levels refers to the number of speed levels from the maximum speed (including) to zero (excluding). So if your maximum speed is 3 m/s and your speed granularity is 3, then your action space will contain speed settings of 1 m/s, 2m/s, and 3 m/s. Simply put 3m/s divide 3 = 1m/s, so go from 0m/s to 3m/s in increments of 1m/s. 0m/s is not included in the action space.

Based on the above example the final action space will include 15 discrete actions (3 speeds x 5 steering angles), that should be listed in the AWS DeepRacer service. If you haven't done so please configure your action space. Feel free to use what you want to use. Larger action spaces may take a bit longer to train.

Hints

- Your model will not perform an action that is not in the action space. Similarly, if your model is trained on a track that that never required the use of this action, for example turning won't be incentivized on a straight track, the model won't know how to use this action as it won't be incentivized to turn. Thus as you start thinking about building a robust model make sure you keep the action space and training track in mind.  
- Specifying a fast speed or a wide steering angle is great, but you still need to think about your reward function and whether it makes sense to drive full-speed into a turn, or exhibit zig-zag behavior on a straight section of the track.
- Our experiments have shown that models with a faster maximum speed take longer to converge than those with a slower maximum speed.
- For real world racing you will have to play with the speed in the webserver user interface of AWS DeepRacer to make sure your car is not driving faster than what it learned in the simulator.


## 3.4 報酬関数
強化学習において、報酬関数はモデルをトレーニングする上で**非常に重要な**役割をもっています。報酬関数は、トレーニングモデルが行動決定を行う際にとってほしい行動に報酬を与えるために使われます。

報酬関数は、ある行動から得られる結果を評価し、その行動に報酬を与えます。実際には、報酬はトレーニング中、各行動が取られた後に計算され、経験という形（ステート、アクション、次のステート、報酬）でモデルのトレーニングに使われます。報酬関数のロジックはシミュレータによって提供される変数によって構築することができます。これらの変数は車からの測定値を表しており、例えば、ステアリング角度、スピード、レーストラック上での(X, Y)座標や経路情報となります。これらの測定値を使い、独自の報酬関数ロジックをPython 3シンタックスを利用して実装することができます。

次のテーブルは、報酬関数で利用できる変数を表しています。これらの変数は、エンジニアやサイエンティストがよりよい方法を見つけるたびにアップデートされるため、適時作った報酬関数を調整するよう注意してください。Singapore Summit (2019年4月10日)時点での変数名・説明はAWSコンソールと同じです。AWS DeepRacer サービス内の最新の情報を常に使うようにしてください。SageMaker RL ノートブックを利用される場合は、ノートブック自体のシンタックスを確認するよう注意してください。


| 変数名        | シンタックス                                                                                | 型                     | 説明                                                                                                                                                                                                                                                                                                                                                         |
|----------------------|---------------------------------------------------------------------------------------|--------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| all_wheels_on_track  | params['all_wheels_on_track']                                                         | Boolean                  | 4輪全てがトラック（走行路または線）上にある場合、all_wheels_on_trackはTrueとなります。1輪でもトラックの外にある場合、all_wheels_on_trackはFalseとなります。                                                                             |
| x                    | params['x']                                                                           | Float                    | 車の前車軸の中心のX座標をメートル単位で返します。                                                                                                                                                                                                                                                                                |
| y                    | params['y']                                                                           | Float                    | 車の前車軸の中心のY座標をメートル単位で返します。                                                                                                                                                                                                                                                                                 |
| distance_from_center | params['distance_from_center']                                                        | Float [0, track_width/2] | トラックの中心からの絶対距離。トラックの中心は全てのwaypointsの中心が繋げられた線により定義されます。                                                                                                                                                                                                                                      |
| is_left_of_center    | params['is_left_of_center']                                                           | Boolean                  | 車がトラックの中心から左側に位置するかどうかを示します。                                                                                                                                                                                                                                                                                     |
| is_reversed          | params['is_reversed']                                                                 | Boolean                  | 車がトラックの順方向でトレーニングしているのか、逆方向でトレーニングしているのか。                                                                                                                                                                                                                                    |
| heading              | params['heading']                                                                     | Float (-180,180]           | 車の先頭の向いている角度を示します。X軸が増加する方向（Y軸は固定）に向いている場合、0を返します。Y軸が増加する方向（X軸は固定）の場合、90を返します。Y軸が減少する方向（X軸は固定）の場合、-90を返します。 |
| progress             | params['progress']                                                                    | Float [0,100]            | 完了したトラックの割合をパーセンテージで示します。100はトラックの完了を示します。                                                                                                                                                                                                                                                                                   |
| steps                | params['steps']                                                                       | Integer [0,inf]                 | 完了したステップを返します。 1ステップは1つのstate, action, next state, rewardのセットを示します。                                                                                                                                                                                                                                                                               |
| speed                | params['speed']                                                                       | Float                    | 期待する車のスピードがメートル/秒で返されます。これは選択されたアクションに結び付けられます。                                                                                                                                                                                                                                                               |
| steering_angle       | params['steering_angle']                                                              | Float                    | 度単位で表される、ステアリングの角度。定義したAction Spaceに紐づきます。※ 正の値は左向きを表し、負の値は右向きを表します。これは2次元平面上で処理されます。                                                                                       |
| track_width          | params['track_width']                                                                 | Float                    | トラックの幅を表します                                                                                                                                                                                                                                                                                                                             |
| waypoints            | params['waypoints'] for the full list or params['waypoints'][i] for the i-th waypoint | List                     | トラックの中心の順序付きリスト。各要素は(x, y)軸を持ち、リストのインデックスは0から始まります。                                                                                                                                                                            |
| closest_waypoints    | params['closest_waypoints'][0] or params['closest_waypoints'][1]                      | Integer                  | 車の現在地から最も近いwaypointのインデックス。params['closest_waypoints'][0]は後方のインデックスを、params['closest_waypoints'][1]は前方のインデックスを示します。                                                                                                          |
|                      |                                                                                       |                          |                                                                                                                                                                                                                                                                                                                                                                     |
|                      |                                                                                       |                          |                                                                                                                                                                                                                                                                                                                                                                     |
|                      |                                                                                       |                          |                                                                                                                                                                                                                                                                                                                                                                     |


以下は報酬関数に利用できるいくつかのパラメータを図示したものです。

![rewardparams](img/reward_function_parameters_illustration.png)

これはre:Inventトラックで使われた、waypointsを可視化した画像です。報酬関数の中では、waypointsの中心線のみアクセスすることができます。このグラフは、報酬関数内でwaypointsのリストを出力し、プロットすることで自身で作ることができます。print関数を報酬関数内で利用する場合、出力はAWS RoboMaker logsに置かれます。どのトラックにおいても行うことができます。詳細は後の方に記します。

![waypoints](img/reinventtrack_waypoints.png)

報酬関数を考える上で役立つ方法は、車を上手に走らせるにはあなたならどうするのかを考えることです。簡単な例は、路上に留まり続ける車に対して報酬を与えるものです。これはreward = 1とすることで行うことができます。DeepRacerのシミュレーター上では、車がトラックの外に出た場合にはリセットし、トラックを再スタートするため、トラックを外れる行動に対して報酬を考える必要はありません。しかし、この報酬関数は、関数内部で利用できる他の全ての変数を全く利用していないため、おそらく最も優れた報酬関数とは言えません。

以下にいくつかの報酬関数の例を示します。

**Example 1**:中心線に沿った動きに報酬を与える基本的な報酬関数
ここでは、3つのマーカーによって、3つの走行帯をトラックの周りに作ります。そして、狭い走行帯を走る車により高い報酬を与えます。報酬の大きさには違いがあり、狭い走行帯で走るものには、1を、真ん中の走行帯で走るものには0.5を、一番広い走行帯を走るものには0.1を与えます。もし、狭い走行帯の報酬を減らすか、真ん中の走行帯の報酬を増やすと、トラックの走行面を広く使うように報酬を与える形になります。これは、特に鋭いカーブに有効です。


	def reward_function(params):
		'''
		Example of rewarding the agent to follow center line
		'''
		
		# Calculate 3 marks that are farther and father away from the center line
		marker_1 = 0.1 * params['track_width']
		marker_2 = 0.25 * params['track_width']
		marker_3 = 0.5 * params['track_width']
		
		# Give higher reward if the car is closer to center line and vice versa
		if params['distance_from_center'] <= marker_1:
			reward = 1.0
		elif params['distance_from_center'] <= marker_2:
			reward = 0.5
		elif params['distance_from_center'] <= marker_3:
			reward = 0.1
		else:
			reward = 1e-3  # likely crashed/ close to off track
		
		return float(reward)

ヒント: 0に等しい報酬を与えないでください。報酬を小さな値で初期化しているため、オプティマイザーは報酬に0を与えられると混乱します。 

**Example 2**:極端なステアリングにペナルティーを与え、中心線に沿った動きに報酬を与える高度な報酬関数


	def reward_function(params):
		'''
		Example that penalizes steering, which helps mitigate zig-zag behaviors
		'''

		# Calculate 3 marks that are farther and father away from the center line
		marker_1 = 0.1 * params['track_width']
		marker_2 = 0.25 * params['track_width']
		marker_3 = 0.5 * params['track_width']

		# Give higher reward if the car is closer to center line and vice versa
		if params['distance_from_center'] <= marker_1:
			reward = 1
		elif params['distance_from_center'] <= marker_2:
			reward = 0.5
		elif params['distance_from_center'] <= marker_3:
			reward = 0.1
		else:
			reward = 1e-3  # likely crashed/ close to off track

		# Steering penality threshold, change the number based on your action space setting
		ABS_STEERING_THRESHOLD = 15

		# Penalize reward if the car is steering too much
		if abs(params['steering_angle']) > ABS_STEERING_THRESHOLD:  # Only need the absolute steering angle
			reward *= 0.5

		return float(reward)
		

**Example 3**:速度を落とすのにペナルティーを与え、中心線に沿った動きに報酬を与える高度な報酬関数


	def reward_function(params):
		'''
		Example that penalizes slow driving. This create a non-linear reward function so it may take longer to learn.
		'''

		# Calculate 3 marks that are farther and father away from the center line
		marker_1 = 0.1 * params['track_width']
		marker_2 = 0.25 * params['track_width']
		marker_3 = 0.5 * params['track_width']

		# Give higher reward if the car is closer to center line and vice versa
		if params['distance_from_center'] <= marker_1:
			reward = 1
		elif params['distance_from_center'] <= marker_2:
			reward = 0.5
		elif params['distance_from_center'] <= marker_3:
			reward = 0.1
		else:
			reward = 1e-3  # likely crashed/ close to off track

		# penalize reward for the car taking slow actions
		# speed is in m/s
		# the below assumes your action space has a maximum speed of 5 m/s and speed granularity of 3
		# we penalize any speed less than 2m/s
		SPEED_THRESHOLD = 2
		if params['speed'] < SPEED_THRESHOLD:
			reward *= 0.5

		return float(reward)

これらのサンプルを利用し、自身の報酬関数を作りましょう。以下に他のTipsを示します。

- waypointsを利用し、あるwaypointと次のwaypointから方向を計算することができます。
- 2Dゲームにおける右手の法則を利用することで、トラックのどっち側にいるか判断することができます。
- 指数的に報酬を与えることができますが、最大10,000に制限してください。
- 報酬関数でspeedとsteering_angleを利用する際は、Action Spaceを覚えておいてください。
- ログ内で、ラップを完走したepisodeを追跡するには、progress = 100の際に、完走ボーナス(つまりは reard += 10000)を与えることを考慮にいれてください。これは、一度車がラップを完了した後は、progressは100以上にはなりませんが、シミュレーションは続くためです。モデルは終了時間に到達するまでトレーニングを続けますが、特に実際のレースでは、最終モデルがベストなモデルとは言えないためです。これは一時的なワークアラウンドになります。

報酬関数を作り終えたら、**Validate**ボタンを押し、コードシンタックスが正しいことをモデルのトレーニング前に確認してください。トレーニングを開始すると、報酬関数はファイルとしてS3に保存されまずが、念のためコピーし、他の場所へ保存してください。

これが、1つめの例を利用した報酬関数の例になります。

![rewardfunction](img/NewReward.png)

次のセクションにスクロールしてください。

## 3.5 Algorithm settings
This section specifies the hyperparameters that will be used by the reinforcement learning algorithm during training. Hyperparameters are used to improve training performance.

Before we dive in, let's just call out some terms we will be using to ensure you are familiar with what they mean.

A **step**, also known as experience, is a tuple of (s,a,r,s’), where s stands for an observation (or state) captured by the camera, a for an action taken by the vehicle, r for the expected reward incurred by the said action, and s’ for the new observation (or new state) after the action is taken.

An **episode** is a period in which the vehicle starts from a given starting point and ends up completing the track or going off the track. Thus an episode is a sequence of steps, or experience. Different episodes can have different lengths.

An **experience buffer** consists of a number of ordered steps collected over fixed number of episodes of varying lengths during training. It serves as the source from which input is drawn for updating the underlying (policy and value) neural networks.

A **batch** is an ordered list of experiences, representing a portion of the experience obtained in the simulation over a period of time, and it is used to update the policy network weights.

A set of batches sampled at random from an experience buffer is called a **training data set**, and used for training the policy network weights.

Our scientists have tested these parameters a lot, and based on the re:Invent track and a small action space these parameters work well, so feel free to leave them unchanged. However, consider changing them as you start iterating on your models as they can significantly improve training convergence.

| Parameter                                | Description                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      |
|------------------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| Batch size                               | The number recent of vehicle experiences sampled at random from an experience buffer and used for updating the underlying deep-learning neural network weights.   If you have 5120 experiences in the buffer, and specify a batch size of 512, then ignoring random sampling, you will get 10 batches of experience. Each batch will be used, in turn, to update your neural network weights during training.  Use a larger batch size to promote more stable and smooth updates to the neural network weights, but be aware of the possibility that the training may be slower. |
| Number of epochs                         |  An epoch represents one pass through all batches, where the neural network weights are updated after each batch is processed, before proceeding to the next batch.   10 epochs implies you will update the neural network weights, using all batches one at a time, but repeat this process 10 times.  Use a larger number of epochs to promote more stable updates, but expect slower training. When the batch size is small,you can use a smaller number of epochs.                                                                                                           |
| Learning rate                            |  The learning rate controls how big the updates to the neural network weights are. Simply put, when you need to change the weights of your policy to get to the maximum cumulative reward, how much should you shift your policy.  A larger learning rate will lead to faster training, but it may struggle to converge. Smaller learning rates lead to stable convergence, but can take a long time to train.                                                                                                                                                                   |
| Exploration                              |  This refers to the method used to determine the trade-off between exploration and exploitation. In other words, what method should we use to determine when we should stop exploring (randomly choosing actions) and when should we exploit the experience we have built up.  Since we will be using a discrete action space, you should always select CategoricalParameters.                                                                                                                                                                                                   |
| Entropy                                  | A degree of uncertainty, or randomness, added to the probability distribution of the action space. This helps promote the selection of random actions to explore the state/action space more broadly.                                                                                                                                                                                                                                                                                                                                                                            |
| Discount factor                          | A factor that specifies how much the future rewards contribute to the expected cumulative reward. The larger the discount factor, the farther out the model looks to determine expected cumulative reward and the slower the training. With a discount factor of 0.9, the vehicle includes rewards from an order of 10 future steps to make a move. With a discount factor of 0.999, the vehicle considers rewards from an order of 1000 future steps to make a move. The recommended discount factor values are 0.99, 0.999 and 0.9999.                                         |
| Loss type                                | The loss type specified the type of the objective function (cost function) used to update the network weights. The Huber and Mean squared error loss types behave similarly for small updates. But as the updates become larger, the Huber loss takes smaller increments compared to the Mean squared error loss. When you have convergence problems, use the Huber loss type. When convergence is good and you want to train faster, use the Mean squared error loss type.                                                                                                      |
| Number of episodes between each training |  This parameter controls how much experience the car should obtain between each model training iteration. For more complex problems which have more local maxima, a larger experience buffer is necessary to provide more uncorrelated data points. In this case, training will be slower but more stable. The recommended values are 10, 20 and 40.                           |

Note that after each training iteration we will save the new model file to your S3 bucket. The AWS DeepRacer service will not show all models trained during a training run, just the last model. We will look at these in Section 3.

## 3.6 Stop conditions
This is the last section before you start training. Here you can specify the maximum time your model will train for. Ideally you should put a number in this condition. You can always stop training early. Furthermore, if your model stopped as a result of the condition, you can go to the model list screen, and clone your model to restart training using new parameters.

Please specify 90 minutes and then select **Start training**. If there is an error, you will be taken to the error location. Note the Python syntax will also be validated again. Once you start training it can take up to 6 minutes to spin up the services needed to start training. During this time let's talk through the AWS DeepRacer League and how you can take part.

![Stopping conditions](img/stop_conditions.png)

Note 25 to 35 minutes of lab time should have elapsed by this point.

Hint: Please make sure you save your reward function, and download your trained model from the burner account. You will lose access to the account after the Summit, and the account will be wiped.


# Section 2: Competing in the AWS DeepRacer League
The [AWS DeepRacer League](https://aws.amazon.com/deepracer/league/) is the world's first global autonomous racing league. The League will take place in 2019 in-person, at various selected locations, and online. Race and you stand to win one of many AWS DeepRacer prizes, or one of 47 paid trips to re:Invent 2019 where you will get to take part in the AWS DeepRacer Knockout Rounds. If you make it through the Knockouts you will get to race in the AWS DeepRacer Championship Cup. [Terms and conditions insert link here]() apply.

The in-person races are referred to as the Summit Circuit, and the online races are referred to as the Virtual Circuit. The locations of the Summit Circuit events can be found [here](https://aws.amazon.com/deepracer/summit-circuit/). The details of the Virtual Circuit will be announced when the AWS DeepRacer service is opened for general availability. You don't need to own an AWS DeepRacer to take part in either form of competition.


![League](img/league.png)


## Racing in the Summit Circuit
To race in the Summit Circuit you must bring your trained AWS DeepRacer RL model to the Summit on a USB stick in a folder called models. Note that we will also provide standard models as part of a walk-up experience for those who were not able to train their own models. At each even you will have to queue for time on the track, on a first come first serve basis, or as the Summit organizer determined, and have 4 minutes to try and get the best lap time using your model and a standard AWS DeepRacer car that we will make available to race with on the race track. The race track will be the re:Invent 2018 track, so train your model on the re:Invent track if you intend to race at any of the selected AWS Summits. The fastest racer at each race in the Summit Circuit will proceed to re:Invent and the top 10 at each race will win AWS DeepRacer cars.

## Racing in the Virtual Circuit
To race in the Virtual Circuit you will have to enter your models into each race, by submitting them via the AWS DeepRacer service in the AWS console. Virtual Circuit races can be seen in the [DeepRacer Virtual Circuit](https://console.aws.amazon.com/deepracer/home?region=us-east-1#leaderboards) section in the AWS DeepRacer service.

![VirtualCircuit](img/dvc.png)

Scroll down for a list of open races

![VirtualCircuitOpen](img/dvc-ll.png)

To see more info on the race, select race information.

![VirtualCircuitInfo](img/dvc-info.png)

Once you have a trained model, you can  submit it into the current open race. Your model will then be evaluated by the AWS DeepRacer service on the indicated competition track. After your model has been evaluated you will see your standing update if your lap time was better than your prior submission.

![VirtualCircuitModelSubmit](img/model-submitted.png)

Each race in the Virtual Circuit will have its own new competition track and it won't be possible to directly train on the competition tracks. Instead we will make a track available that will be similar in theme and design to each competition track, but not identical. This ensures that models have to generalize, and can't just be overfitted to the competition track. The fastest racer in each race in the Virtual Circuit will proceed to re:Invent and the top 10 at each race will win AWS DeepRacer cars.

**Tip**: The DeepRacer service does not currently support importing models,  but you can still save your model.tar.gz file, as well as all model training artifacts. The final model is stored as model.tar.gz file in a folder called DeepRacer-SageMaker-rlmdl-account number-date in your DeepRacer S3 bucket. The interim models are stored as .pd files in a folder called DeepRacer-SageMaker-RoboMaker-comm-account number-date

After each event in the Summit Circuit and in the Virtual Circuit, all racers that took part will receive points based on the time it took them to complete the race. Points will aggregate through the season, and at the end of the seasons the top point getters will be invited to take part at re:Invent. Please refer to the [terms and conditions insert link here](https://aws.amazon.com/deepracer/faqs/#AWS_DeepRacer_League) for more details. 

# Section 3: Model training and improving your model

## 3.1: While your model is training

After your model has started training you can select it from the listed models. You can then see the how the training is progressing by looking at the total reward per episode graph, and also at the first person view from the car in the simulator. 

At first your car will not be able to drive on a straight road but as it learns better driving behavior you should see its performance improving, and the reward graph increasing. Furthermore, when you car drives off of the track it will be reset on the track. Don't be alarmed if your car doesn't start at the same position. We have enabled round robin to allow the car to start at subsequent points on the track each time to ensure it can train on experience from the whole track. Furthermore, during training you may see your car start training in the opposite direction of the track. This is also done to ensure the model generalizes better, and is not caught off guard by an asymmetrical count between left and right hand turns. Lastly, if you see your car aimlessly drive off track and not resetting, this is when the experience obtained is sent back to Amazon SageMaker to train the model. Once the model has been updated, the new model will be sent back to AWS RoboMaker and the car will resume.

You can look at the log files for both Amazon SageMaker and AWS RoboMaker. The logs are outputted to Amazon CloudWatch. To see the logs, hover your mouse over the reward graph and select the three dots that appear below the refresh button, please then select **View logs**.

![Training Started](img/widg.png)

You will see the logs of the Python validation lambda, Amazon SageMaker, and AWS RoboMaker.

![Logs](img/view_in_logs.png)

Each folder will contain the logs for all training jobs that you have executed in AWS DeepRacer. AWS RoboMaker logs will contain output from the simulator, and the Amazon SageMaker logs will contain output from the model training. If there are any errors, the logs are a good place to start.

![AWS RoboMaker Logs](img/robomaker_logs.png)

The AWS DeepRacer service makes use of Amazon SageMaker, AWs RoboMaker, Amazon S3, Amazon Kinesis Video Streams, AWS Lambda, and Amazon CloudWatch. You can navigate to each of these services to get an update on the service's status or for other useful information.

In each service you will see a list of current and prior jobs, where retained. Here is a view of training jobs executed in Amazon SageMaker.

![SageMaker jobs](img/sagemaker_listjobs.png)

In Amazon SageMaker you will be able to see the logs as well as utilization of the EC2 instance spun up to run training.

![SageMaker jobs](img/sagemaker_jobdetails.png)

In AWS RoboMaker you can see the list of all simulation jobs, and for active jobs you can get a direct view into the simulation environment.

![RoboMaker jobs](img/aws_robomaker_jobs_list.png)

You can select your active simulation job from the list and then select the Gazebo icon. 

![RoboMaker job details](img/aws_robomaker.png)

This will open a new window showing you the simulation environment. **Take care in this environment because any changes you make to it will affect your simulation in real time. Thus if you accidentally drag or rotate the vehicle or the environment, it may negatively affect your training job.**

![RoboMaker simulator](img/robomaker_simulator.png) 

The Amazon Kinesis Video Stream is typically deleted after use to free up space and due to limits on the number of streams. Note also that at present the video is not yet stored in your S3 account, for both training and evaluations.

![KVS stream](img/kvs_stream_video.png)

Amazon S3 will store the final model, that is referenced in the AWS DeepRacer service, and interim models trained during your training jobs in the aws-deepracer bucket. Your reward functions will also be stored in the same bucket.

![S3list](img/s3.png)

The final model is stored as model.tar.gz file in a folder called DeepRacer-SageMaker-rlmdl-account number-date in your DeepRacer S3 bucket.  
The interim models are stored as .pd files in a folder called DeepRacer-SageMaker-RoboMaker-comm-account number-date
![S3dr](img/s3_aws_deepracer.png)

The AWS DeepRacer service can at the time of writing only reference one final model for each training job. However, should you want to swap out the model trained during the final training iteration, with any model trained in the training iterations running up to the final, you can simple swap out the model.pb file in the final model.tar.gz file. Note that you should not change the other files in the .tar.gz as this may render the model useless. Do this after your model has stopped training, or after you manually stopped training.

## 3.2: Evaluating the performance of your model

You may not have time in the workshop to do from step 2 onwards. Once your model training is complete you can start model evaluation. From your model details page, where you observed training, select **Start evaluation**. You can now select the track on which you want to evaluate the performance of your model and also the number of laps. Select the re:Invent 2018 track and 5 laps and select Start. 

Once done you should see something as follows.

![evaluation_done](img/evaluation_done.png)

## 3.3: Race in the AWS DeepRacer League

If you are happy with your model you can go race in the [Summit Circuit](https://aws.amazon.com/deepracer/summit-circuit/) or right now in the [Virtual Circuit](https://console.aws.amazon.com/deepracer/home?region=us-east-1#leaderboards). You can submit your trained model into the Virtual Circuit's current open race [here](https://console.aws.amazon.com/deepracer/home?region=us-east-1#leaderboards).

## 3.4: Iterating and improving your model

Based on the evaluation of the model you should have a good idea as to whether your model can complete the track reliably, and what the average lap time is. Note that for the Virtual Circuit races you will have to complete a certain number of laps consecutively with your model, and so focus on building a reliable model. The number of laps will be determined race by race.

At this point you have to experiment and iterate on your reward function and hyperparameters. It is best to try a few different reward functions based on different driving behavior, and then evaluate them in the simulator to select the best performing one. If you have an AWS DeepRacer you can also test them in the real world.

Hints:
- Increase training time beyond. If your model can't reliably complete a lap try to extend your model training time.
- Try modifying action space by increasing max speed to get faster lap times.
- Tweak your reward function to incentivize your car to drive faster : you’ll want to specifically modify progress, steps and speed variables.
- Clone your model to leverage training experience. Please note that you will not be able to change action space once a model is cloned, otherwise the job will fail.

## 3.5: Analyze model performance by inspecting the RoboMaker logs
If you do want to go a step further, you can evaluate the performance of each model that was trained during the training job by inspecting the log file.

To download the log file from CloudWatch you can use the following code with [Amazon CLI](https://docs.aws.amazon.com/polly/latest/dg/setup-aws-cli.html).  

**Download the RoboMaker log from CloudWatch**


1. [Quick Analysis] Get last 10000 lines from the log

	aws logs get-log-events --log-group-name  "/aws/robomaker/SimulationJobs"  --log-stream-name  "<STREAM_NAME>" --output text --region us-east-1 > deepracer-sim.log

2. [Export Entire Log] Copy the log from Amazon Cloudwatch to Amazon S3. Follow the link to export all the logs to [Amazon S3](https://docs.aws.amazon.com/AmazonCloudWatch/latest/logs/S3ExportTasks.html)

You can now analyze the log file using Python Pandas and see which model iterations provided the highest total reward. Furthermore, if you did add a finish bonus, you can see which model iterations were able to finish a lap. These models are good candidates to test in the simulator and in the real world.




