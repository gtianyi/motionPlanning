from textx.metamodel import metamodel_from_str

grammar = """
Configuration: attributes*=Attribute;
Attribute: key=/[a-zA-Z0-9]+/ '?' value=/([a-zA-Z0-9-_]|[\/]|[*]|[.]|[ ])+/ '\n';
"""

configuration_metamodel = metamodel_from_str(grammar)


def parse_configuration(raw_configuration):
    configuration = {}
    model = configuration_metamodel.model_from_str(raw_configuration)
    for attribute in model.attributes:
        configuration[attribute.key] = attribute.value

    return configuration
