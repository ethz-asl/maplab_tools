import ray
from ray import tune
import functools
from ray.tune import Trainable

class Guesser(Trainable):
    def _setup(self, config):
        self.config = config
        self.password = 1024

    def _train(self):
        result_dict = {"diff": abs(self.config['guess'] - self.password)}
        return result_dict

def objective(step, alpha, beta):
    return (0.1 + alpha * step / 100)**(-1) + beta * 0.1


def training_function(config, a):
    # Hyperparameters
    alpha, beta = config["alpha"], config["beta"]
    for step in range(10):
        # Iterative training function - can be any arbitrary training procedure.
        intermediate_score = objective(step, alpha, beta)
        # Feed the score back back to Tune.
        # tune.report(mean_loss=intermediate_score)
        tune.track.log(mean_loss=intermediate_score)

# f = functools.partial(training_function, a=1)
# analysis = tune.run(
#     f,
#     config={
#         "alpha": tune.grid_search([0.001, 0.01, 0.1]),
#         "beta": tune.choice([1, 2, 3])
#     })
analysis = tune.run(
    Guesser,
    stop={
        "training_iteration": 1,
    },
    num_samples=1,
    config={
        "guess": tune.randint(1, 10000)
    })

print("Best config: ", analysis.get_best_config(
    metric="mean_loss", mode="min"))

# Get a dataframe for analyzing trial results.
df = analysis.dataframe()
print(df)
