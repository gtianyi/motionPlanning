def forest_environment():
    base = 'EnvironmentMesh ? forest.dae\n' \
           '{start_goal_pair}\n' \
           'EnvironmentBounds ? -30 30 -30 30\n' \
           'EnvironmentName ? 2D-forest\n'

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
           'EnvironmentBounds ? -15 15 -15 15\n' \
           'EnvironmentName ? ladder\n'

    start_goal_pairs = [
        "Start ? -13 -12 \nGoal ? -13 12",
        "Start ? -13 -13 \nGoal ? -13 12",
        "Start ? -13 -11 \nGoal ? -13 12",
        "Start ? -13 -12 \nGoal ? -13 13",
        "Start ? -13 -12 \nGoal ? -13 11",
    ]

    return [base.format(start_goal_pair=start_goal_pair) for start_goal_pair in start_goal_pairs]


def wall_environment():
    base = 'EnvironmentMesh ? single-wall.dae\n' \
           '{start_goal_pair}\n' \
           'EnvironmentBounds ? -15 15 -15 15\n' \
           'EnvironmentName ? single-wall\n'

    start_goal_pairs = [
        "Start ? -13 -12 \nGoal ? -13 13",
        "Start ? -13 -13 \nGoal ? -12 10",
        "Start ? -13 -11 \nGoal ? -10 10",
        "Start ? -10 -12 \nGoal ? -13 13",
        "Start ? -5 -12 \nGoal ? -13 13",
    ]

    return [base.format(start_goal_pair=start_goal_pair) for start_goal_pair in start_goal_pairs]


def empty_environment():
    base = 'EnvironmentMesh ? single-wall.dae\n' \
           '{start_goal_pair}\n' \
           'EnvironmentBounds ? -15 15 -10 15\n' \
           'EnvironmentName ? empty-area\n'

    start_goal_pairs = [
        "Start ? -13 7 \nGoal ? 13 7",
        "Start ? -13 8 \nGoal ? 12 8",
        "Start ? -12 7 \nGoal ? 10 7",
        "Start ? -10 5 \nGoal ? 10 6",
        "Start ? -7 7 \nGoal ? 7 8",
    ]

    return [base.format(start_goal_pair=start_goal_pair) for start_goal_pair in start_goal_pairs]

def threed_forest_environment():
    base = 'EnvironmentMesh ? forest.dae\n' \
           '{start_goal_pair}\n' \
           'EnvironmentBounds ? -30 30 -30 30 -5 16\n' \
           'EnvironmentName ? 3D-forest\n'

    start_goal_pairs = [
        "Start ? 0 0 15 \nGoal ? 0 -25 0",
        "Start ? 0 0 15 \nGoal ? 0 -26 0",
        "Start ? 0 0 15 \nGoal ? 0 -26 -1",
        "Start ? 0 0 15 \nGoal ? 1 -25 1",
        "Start ? 0 0 15 \nGoal ? 5 -25 -3"
    ]

    return [base.format(start_goal_pair=start_goal_pair) for start_goal_pair in start_goal_pairs]


def threed_fifthelement_environment():
    base = 'EnvironmentMesh ? fifthelement.dae\n' \
           '{start_goal_pair}\n' \
           'EnvironmentBounds ? -30 30 -30 30 0 15\n' \
           'EnvironmentName ? fifthelement\n'

    start_goal_pairs = [
        "Start ? 15 7 5 \nGoal ? -15 7 5",
        "Start ? 15 -7 5\nGoal ? -15 -7 5",
        "Start ? 15 -7 5\nGoal ? -15 7 5",
        "Start ? 15 7 5\nGoal ? -15 -7 5",
        "Start ? 7 15 5\nGoal ? -7 -15 5",
    ]

    return [base.format(start_goal_pair=start_goal_pair) for start_goal_pair in start_goal_pairs]

def dynamic_car_domain():
    base = "Domain ? DynamicCar\n" \
           "AgentMesh ? car2_planar_robot.dae\n" \
           "{environment}\n"

    environments = forest_environment() + ladder_environment() + empty_environment()

    return [base.format(environment=environment) for environment in environments]


def kinematic_car_domain():
    base = "Domain ? KinematicCar\n" \
           "AgentMesh ? car2_planar_robot.dae\n" \
           "{environment}\n"

    environments = forest_environment() + ladder_environment() + empty_environment()

    return [base.format(environment=environment) for environment in environments]


def hovercraft_domain():
    base = "Domain ? Hovercraft\n" \
           "AgentMesh ? car2_planar_robot.dae\n" \
           "{environment}\n"

    environments = forest_environment() + ladder_environment() + empty_environment()

    return [base.format(environment=environment) for environment in environments]


def quadrotor_domain():
    base = "Domain ? Quadrotor\n" \
           "AgentMesh ? quadrotor.dae\n" \
           "{environment}\n"

    environments = threed_forest_environment() + threed_fifthelement_environment()

    return [base.format(environment=environment) for environment in environments]


def blimp_domain():
    base = "Domain ? Blimp\n" \
           "AgentMesh ? blimp.dae\n" \
           "{environment}\n"

    environments = threed_forest_environment() + threed_fifthelement_environment()

    return [base.format(environment=environment) for environment in environments]


def seeded_domains():
    base = "{domain}\n" \
           "Seed ? {seed}"

    domains = dynamic_car_domain() + kinematic_car_domain() + \
          hovercraft_domain() + quadrotor_domain() + blimp_domain()

    # domains=kinematic_car_domain()

    seeds = range(25)

    seeded_domains = []
    for domain in domains:
        seeded_domains += [base.format(seed=seed + 1, domain=domain) for seed in seeds]

    return seeded_domains


def planners():
    # base = "Planner ? {planner}\n" \
    #        "AbstractionType ? PRM\n" \
    #        "WhichSearch ? D*\n"
    #
    # algorithms = ['BEAST', 'BEAST']
    #beast = "Planner ? BEAST\n" \
    #        "AbstractionType ? PRM\n" \
    #        "WhichSearch ? D*\n" \
    #        'ValidEdgeDistributionAlpha ? 10\n' \
    #        'ValidEdgeDistributionBeta ? 1\n' \
    #        'InvalidEdgeDistributionAlpha ? 1\n' \
    #        'InvalidEdgeDistributionBeta ? 10\n' \
    #        "Algorithm ? BEAST"

    # alloneone = "Planner ? BEAST\n" \
    #                   "AbstractionType ? PRM\n" \
    #                   "WhichSearch ? D*\n" \
    #                   'ValidEdgeDistributionAlpha ? 1\n' \
    #                   'ValidEdgeDistributionBeta ? 1\n' \
    #                   'InvalidEdgeDistributionAlpha ? 1\n' \
    #                   'InvalidEdgeDistributionBeta ? 1\n' \
    #                   "Algorithm ? AllOneOne"

    # nogeometric = "Planner ? BEAST\n" \
    #                   "AbstractionType ? PRM\n" \
    #                   "WhichSearch ? D*NOGEOMETRIC\n" \
    #                   'ValidEdgeDistributionAlpha ? 1\n' \
    #                   'ValidEdgeDistributionBeta ? 1\n' \
    #                   'InvalidEdgeDistributionAlpha ? 1\n' \
    #                   'InvalidEdgeDistributionBeta ? 1\n' \
    #                   "Algorithm ? AllOneOne"

    # halton = "Planner ? BEAST\n" \
             # "AbstractionType ? PRM_HALTON\n" \
             # "WhichSearch ? D*\n" \
             # 'ValidEdgeDistributionAlpha ? 10\n' \
             # 'ValidEdgeDistributionBeta ? 1\n' \
             # 'InvalidEdgeDistributionAlpha ? 1\n' \
             # 'InvalidEdgeDistributionBeta ? 10\n' \
             # "Algorithm ? Haltonfix"

    # grid = "Planner ? BEAST\n" \
            # "AbstractionType ? PRM_GRID\n" \
            # "WhichSearch ? D*\n" \
            # 'ValidEdgeDistributionAlpha ? 10\n' \
            # 'ValidEdgeDistributionBeta ? 1\n' \
            # 'InvalidEdgeDistributionAlpha ? 1\n' \
            # 'InvalidEdgeDistributionBeta ? 10\n' \
            # "Algorithm ? GRID"

    # newbonus = "Planner ? BEAST\n" \
    #            "AbstractionType ? PRM\n" \
    #            "WhichSearch ? D*BONUS\n" \
    #            'ValidEdgeDistributionAlpha ? 1\n' \
    #            'ValidEdgeDistributionBeta ? 1\n' \
    #            'InvalidEdgeDistributionAlpha ? 1\n' \
    #            'InvalidEdgeDistributionBeta ? 1\n' \
    #            "Algorithm ? NewBonus"

    # halton_nb = "Planner ? BEAST\n" \
    #             "AbstractionType ? PRM_HALTON\n" \
    #             "WhichSearch ? D*BONUS\n" \
    #             'ValidEdgeDistributionAlpha ? 1\n' \
    #             'ValidEdgeDistributionBeta ? 1\n' \
    #             'InvalidEdgeDistributionAlpha ? 1\n' \
    #             'InvalidEdgeDistributionBeta ? 1\n' \
    #             "Algorithm ? HaltonNB"

    #gust = "Planner ? GUST\n" \
    #       "AbstractionType ? PRM_GUST\n" \
    #       "WhichSearch ? D*\n" \
    #       'ValidEdgeDistributionAlpha ? 10\n' \
    #       'ValidEdgeDistributionBeta ? 1\n' \
    #       'InvalidEdgeDistributionAlpha ? 1\n' \
    #       'InvalidEdgeDistributionBeta ? 10\n' \
    #       'Alpha ? 8\n' \
    #       'B ? 0.85\n' \
    #       'Beta ? 0.85\n' \
    #       'Delta ? 0.15\n' \
    #       'UseSplit ? 1\n' \
    #       'Algorithm ? GUST'
    #beast_int_b0 = "Planner ? BEAST\n" \
    #        "AbstractionType ? PRM\n" \
    #        "WhichSearch ? Integrated\n" \
    #        'ValidEdgeDistributionAlpha ? 10\n' \
    #        'ValidEdgeDistributionBeta ? 1\n' \
    #        'InvalidEdgeDistributionAlpha ? 1\n' \
    #        'InvalidEdgeDistributionBeta ? 10\n' \
    #        "Algorithm ? BEAST_INT\n" \
    #        "RegionCount ? 1000\n" \
    #        "InitialAlpha ? 1\n" \
    #        "InitialBeta ? 1\n" \
    #        "BonusType ? 0\n" \
    #        "HaltonSampling ? false\n" \
    #        "AbstractStateRadius ? 6\n" \
    #        "UseSplit ? false\n" 
    #beast_int_b1 = "Planner ? BEAST\n" \
    #        "AbstractionType ? PRM\n" \
    #        "WhichSearch ? Integrated\n" \
    #        'ValidEdgeDistributionAlpha ? 10\n' \
    #        'ValidEdgeDistributionBeta ? 1\n' \
    #        'InvalidEdgeDistributionAlpha ? 1\n' \
    #        'InvalidEdgeDistributionBeta ? 10\n' \
    #        "Algorithm ? BEAST_INT\n" \
    #        "RegionCount ? 1000\n" \
    #        "InitialAlpha ? 1\n" \
    #        "InitialBeta ? 1\n" \
    #        "BonusType ? 1\n" \
    #        "HaltonSampling ? false\n" \
    #        "AbstractStateRadius ? 6\n" \
    #        "UseSplit ? false\n" 
    #beast_int_b2 = "Planner ? BEAST\n" \
    #        "AbstractionType ? PRM\n" \
    #        "WhichSearch ? Integrated\n" \
    #        'ValidEdgeDistributionAlpha ? 10\n' \
    #        'ValidEdgeDistributionBeta ? 1\n' \
    #        'InvalidEdgeDistributionAlpha ? 1\n' \
    #        'InvalidEdgeDistributionBeta ? 10\n' \
    #        "Algorithm ? BEAST_INT\n" \
    #        "RegionCount ? 1000\n" \
    #        "InitialAlpha ? 1\n" \
    #        "InitialBeta ? 1\n" \
    #        "BonusType ? 2\n" \
    #        "HaltonSampling ? false\n" \
    #        "AbstractStateRadius ? 6\n" \
    #        "UseSplit ? false\n" 
    #beast_int_b3 = "Planner ? BEAST\n" \
    #        "AbstractionType ? PRM\n" \
    #        "WhichSearch ? Integrated\n" \
    #        'ValidEdgeDistributionAlpha ? 10\n' \
    #        'ValidEdgeDistributionBeta ? 1\n' \
    #        'InvalidEdgeDistributionAlpha ? 1\n' \
    #        'InvalidEdgeDistributionBeta ? 10\n' \
    #        "Algorithm ? BEAST_INT\n" \
    #        "RegionCount ? 1000\n" \
    #        "InitialAlpha ? 1\n" \
    #        "InitialBeta ? 1\n" \
    #        "BonusType ? 3\n" \
    #        "HaltonSampling ? false\n" \
    #        "AbstractStateRadius ? 6\n" \
    #        "UseSplit ? false\n" 
    #beast_split = "Planner ? BEAST\n" \
    #        "AbstractionType ? PRM\n" \
    #        "WhichSearch ? Integrated\n" \
    #        'ValidEdgeDistributionAlpha ? 10\n' \
    #        'ValidEdgeDistributionBeta ? 1\n' \
    #        'InvalidEdgeDistributionAlpha ? 1\n' \
    #        'InvalidEdgeDistributionBeta ? 10\n' \
    #        "Algorithm ? BEAST-INT-SPLIT-500\n" \
    #        "AbstractStateRadius ? 6\n" \
    #        "UseSplit ? true\n" \
    #        "RegionCount ? 500\n" \
    #        "InitialAlpha ? 1\n" \
    #        "InitialBeta ? 1\n" \
    #        "BonusType ? 0\n"\
    #        "HaltonSampling ? false\n"

    #beast_split_halton = "Planner ? BEAST\n" \
    #        "AbstractionType ? PRM\n" \
    #        "WhichSearch ? Integrated\n" \
    #        'ValidEdgeDistributionAlpha ? 10\n' \
    #        'ValidEdgeDistributionBeta ? 1\n' \
    #        'InvalidEdgeDistributionAlpha ? 1\n' \
    #        'InvalidEdgeDistributionBeta ? 10\n' \
    #        "Algorithm ? BEAST-INT-SPLIT-HALTON\n" \
    #        "AbstractStateRadius ? 6\n" \
    #        "UseSplit ? true\n" \
    #        "RegionCount ? 1000\n" \
    #        "InitialAlpha ? 1\n" \
    #        "InitialBeta ? 1\n" \
    #        "BonusType ? 0\n"\
    #        "HaltonSampling ? true\n"

    beast_grid = "Planner ? BEAST\n" \
           "AbstractionType ? PRM\n" \
           "WhichSearch ? IntegratedGrid\n" \
           'ValidEdgeDistributionAlpha ? 10\n' \
           'ValidEdgeDistributionBeta ? 1\n' \
           'InvalidEdgeDistributionAlpha ? 1\n' \
           'InvalidEdgeDistributionBeta ? 10\n' \
           "Algorithm ? BEAST-INT-Grid\n" \
           "AbstractStateRadius ? 6\n" \
           "UseSplit ? false\n" \
           "RegionCount ? 1000\n" \
           "InitialAlpha ? 1\n" \
           "InitialBeta ? 1\n" \
           "BonusType ? 0\n"\

    beast_int = "Planner ? BEAST\n" \
           "AbstractionType ? PRM\n" \
           "WhichSearch ? IntegratedPRM\n" \
           'ValidEdgeDistributionAlpha ? 10\n' \
           'ValidEdgeDistributionBeta ? 1\n' \
           'InvalidEdgeDistributionAlpha ? 1\n' \
           'InvalidEdgeDistributionBeta ? 10\n' \
           "Algorithm ? BEAST-INT\n" \
           "AbstractStateRadius ? 6\n" \
           "UseSplit ? false\n" \
           "RegionCount ? 1000\n" \
           "InitialAlpha ? 1\n" \
           "InitialBeta ? 1\n" \
           "BonusType ? 0\n"\
           "HaltonSampling ? false\n"\
           "RejectSampling ? false\n"

    beast_halton = "Planner ? BEAST\n" \
           "AbstractionType ? PRM\n" \
           "WhichSearch ? IntegratedPRM\n" \
           'ValidEdgeDistributionAlpha ? 10\n' \
           'ValidEdgeDistributionBeta ? 1\n' \
           'InvalidEdgeDistributionAlpha ? 1\n' \
           'InvalidEdgeDistributionBeta ? 10\n' \
           "Algorithm ? BEAST-INT-Halton\n" \
           "AbstractStateRadius ? 6\n" \
           "UseSplit ? false\n" \
           "RegionCount ? 1000\n" \
           "InitialAlpha ? 1\n" \
           "InitialBeta ? 1\n" \
           "BonusType ? 0\n"\
           "HaltonSampling ? true\n"\
           "RejectSampling ? false\n"

    beast_split = "Planner ? BEAST\n" \
           "AbstractionType ? PRM\n" \
           "WhichSearch ? IntegratedPRM\n" \
           'ValidEdgeDistributionAlpha ? 10\n' \
           'ValidEdgeDistributionBeta ? 1\n' \
           'InvalidEdgeDistributionAlpha ? 1\n' \
           'InvalidEdgeDistributionBeta ? 10\n' \
           "Algorithm ? BEAST-INT-SPLIT\n" \
           "AbstractStateRadius ? 6\n" \
           "UseSplit ? true\n" \
           "RegionCount ? 1000\n" \
           "InitialAlpha ? 1\n" \
           "InitialBeta ? 1\n" \
           "BonusType ? 0\n"\
           "HaltonSampling ? false\n"\
           "RejectSampling ? false\n"

    beast_grid_500 = "Planner ? BEAST\n" \
           "AbstractionType ? PRM\n" \
           "WhichSearch ? IntegratedGrid\n" \
           'ValidEdgeDistributionAlpha ? 10\n' \
           'ValidEdgeDistributionBeta ? 1\n' \
           'InvalidEdgeDistributionAlpha ? 1\n' \
           'InvalidEdgeDistributionBeta ? 10\n' \
           "Algorithm ? BEAST-INT-Grid-500\n" \
           "AbstractStateRadius ? 6\n" \
           "UseSplit ? false\n" \
           "RegionCount ? 500\n" \
           "InitialAlpha ? 1\n" \
           "InitialBeta ? 1\n" \
           "BonusType ? 0\n"\

    # return [base.format(planner=planner) for planner in algorithms] + [halton]
#return [nogeometric, halton, newbonus, beast]
    return [beast_grid, beast_int, beast_halton] 


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
           'NumEdges ? 5\n' \
           'PRMSize ? 1000\n' \
           'StateRadius ? 6\n' \
           'PRMResizeFactor ? 2\n' \
           

    algorithms = planners()
    domains = seeded_domains()

    configurations = []
    for planner in algorithms:
        configurations += [base.format(planner=planner, domain=domain) for domain in domains]

    return configurations


if __name__ == '__main__':
    print(len(generate_configurations()))
