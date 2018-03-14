def forest_environment():
    base = 'EnvironmentMesh ? forest.dae\n' \
           '{start_goal_pair}\n' \
           'EnvironmentBounds ? -30 30 -30 30\n'

    start_goal_pairs = [
        "Start ? 0 0 \nGoal ? 0 -25",
        "Start ? 0.1 0.1 \nGoal ? 0 -26",
        "Start ? 3 -5 \nGoal ? 0 -25",
        "Start ? -3 -5 \nGoal ? 1 -25",
        "Start ? -5 -5 \nGoal ? 5 -25"
    ]

    return [base.format(start_goal_pair=start_goal_pair) for start_goal_pair in start_goal_pairs]


def ladder_environment():
    base = 'EnvironmentMesh ? 3-ladder.dae\n' \
           '{start_goal_pair}\n' \
           'EnvironmentBounds ? -15 15 -15 15\n'

    start_goal_pairs = [
        "Start ? -13 -12 \nGoal ? -13 12",
        "Start ? -13 -13 \nGoal ? -13 12",
        "Start ? -13 -11 \nGoal ? -13 12",
        "Start ? -13 -12 \nGoal ? -13 13",
        "Start ? -13 -12 \nGoal ? -13 11",
    ]

    return [base.format(start_goal_pair=start_goal_pair) for start_goal_pair in start_goal_pairs]


def dynamic_car_domain():
    base = "Domain ? DynamicCar\n" \
           "AgentMesh ? car2_planar_robot.dae\n" \
           "{environment}\n"

    environments = forest_environment() + ladder_environment()

    return [base.format(environment=environment) for environment in environments]


def kinematic_car_domain():
    base = "Domain ? KinematicCar\n" \
           "AgentMesh ? car2_planar_robot.dae\n" \
           "{environment}\n"

    environments = forest_environment() + ladder_environment()

    return [base.format(environment=environment) for environment in environments]


def seeded_domains():
    base = "{domain}\n" \
           "Seed ? {seed}"

    domains = dynamic_car_domain() + kinematic_car_domain()
    seeds = range(25)

    seeded_domains = []
    for domain in domains:
        seeded_domains += [base.format(seed=seed + 1, domain=domain) for seed in seeds]

    return seeded_domains


def planners():
    base = "Planner ? {planner}\n" \
           "WhichSearch ? D*\n"

    algorithms = ['BEASTnew', 'BEAST']

    return [base.format(planner=planner) for planner in algorithms]


def generate_configurations():
    base = 'Timeout ? 60\n' \
           'Memory ? 1000\n' \
           'AddIntermediateStates ? true\n' \
           'Runs ? 1\n ' \
           '{domain}\n' \
           'PropagationStepSize ? 0.05\n' \
           'MinControlDuration ? 1\n' \
           'MaxControlDuration ? 100\n' \
           'GoalRadius ? 0.1\n' \
           '{planner}\n' \
           'AbstractionType ? PRM\n' \
           'NumEdges ? 5\n' \
           'PRMSize ? 1000\n' \
           'StateRadius ? 6\n' \
           'ValidEdgeDistributionAlpha ? 10\n' \
           'ValidEdgeDistributionBeta ? 1\n' \
           'InvalidEdgeDistributionAlpha ? 1\n' \
           'InvalidEdgeDistributionBeta ? 10\n' 

    algorithms = planners()
    domains = seeded_domains()

    configurations = []
    for planner in algorithms:
        configurations += [base.format(planner=planner, domain=domain) for domain in domains]

    return configurations


if __name__ == '__main__':
    print(len(generate_configurations()))
