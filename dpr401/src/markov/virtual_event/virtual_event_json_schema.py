SINGLE_RACER_INFO_JSON_SCHEMA = {
    "$schema": "http://json-schema.org/draft-04/schema#",
    "type": "object",
    "properties": {
        "racerAlias": {
            "type": "string"
        },
        "carConfig": {
            "type": "object",
            "properties": {
                "carColor": {
                    "type": "string"
                },
                "bodyShellType": {
                    "type": "string"
                }
            }
        },
        "inputModel": {
            "type": "object",
            "properties": {
                "s3BucketName": {
                    "type": "string"
                },
                "s3KeyPrefix": {
                    "type": "string"
                },
                "s3KmsKeyArn": {
                    "type": "string"
                }
            },
            "required": [
                "s3BucketName",
                "s3KeyPrefix"
            ]
        },
        "outputMetrics": {
            "type": "object",
            "properties": {
                "s3BucketName": {
                    "type": "string"
                },
                "s3KeyPrefix": {
                    "type": "string"
                },
                "s3KmsKeyArn": {
                    "type": "string"
                }
            },
            "required": [
                "s3BucketName",
                "s3KeyPrefix"
            ]
        },
        "outputStatus": {
            "type": "object",
            "properties": {
                "s3BucketName": {
                    "type": "string"
                },
                "s3KeyPrefix": {
                    "type": "string"
                },
                "s3KmsKeyArn": {
                    "type": "string"
                }
            },
            "required": [
                "s3BucketName",
                "s3KeyPrefix"
            ]
        },
        "outputSimTrace": {
            "type": "object",
            "properties": {
                "s3BucketName": {
                    "type": "string"
                },
                "s3KeyPrefix": {
                    "type": "string"
                },
                "s3KmsKeyArn": {
                    "type": "string"
                }
            },
            "required": [
                "s3BucketName",
                "s3KeyPrefix"
            ]
        },
        "outputMp4": {
            "type": "object",
            "properties": {
                "s3BucketName": {
                    "type": "string"
                },
                "s3KeyPrefix": {
                    "type": "string"
                },
                "s3KmsKeyArn": {
                    "type": "string"
                }
            },
            "required": [
                "s3BucketName",
                "s3KeyPrefix"
            ]
        }
    },
    "required": [
        "racerAlias",
        "inputModel",
        "outputMetrics",
        "outputStatus",
        "outputSimTrace",
        "outputMp4"
    ]
}

LIST_OF_RACERS_INFO_JSON_SCHEMA = {
    "$schema": "http://json-schema.org/draft-04/schema#",
    "type": "array",
    "minItems": 1,
    "maxItems": 2,
    "additionalProperties": False,
    "items": {
        "properties": {
            "racerAlias": {
                "type": "string"
            },
            "carConfig": {
                "type": "object",
                "properties": {
                    "carColor": {
                        "type": "string"
                    },
                    "bodyShellType": {
                        "type": "string"
                    }
                }
            },
            "inputModel": {
                "type": "object",
                "properties": {
                    "s3BucketName": {
                        "type": "string"
                    },
                    "s3KeyPrefix": {
                        "type": "string"
                    },
                    "s3KmsKeyArn": {
                        "type": "string"
                    }
                },
                "required": [
                    "s3BucketName",
                    "s3KeyPrefix"
                ]
            },
            "outputMetrics": {
                "type": "object",
                "properties": {
                    "s3BucketName": {
                        "type": "string"
                    },
                    "s3KeyPrefix": {
                        "type": "string"
                    },
                    "s3KmsKeyArn": {
                        "type": "string"
                    }
                },
                "required": [
                    "s3BucketName",
                    "s3KeyPrefix"
                ]
            },
            "outputStatus": {
                "type": "object",
                "properties": {
                    "s3BucketName": {
                        "type": "string"
                    },
                    "s3KeyPrefix": {
                        "type": "string"
                    },
                    "s3KmsKeyArn": {
                        "type": "string"
                    }
                },
                "required": [
                    "s3BucketName",
                    "s3KeyPrefix"
                ]
            },
            "outputSimTrace": {
                "type": "object",
                "properties": {
                    "s3BucketName": {
                        "type": "string"
                    },
                    "s3KeyPrefix": {
                        "type": "string"
                    },
                    "s3KmsKeyArn": {
                        "type": "string"
                    }
                },
                "required": [
                    "s3BucketName",
                    "s3KeyPrefix"
                ]
            },
            "outputMp4": {
                "type": "object",
                "properties": {
                    "s3BucketName": {
                        "type": "string"
                    },
                    "s3KeyPrefix": {
                        "type": "string"
                    },
                    "s3KmsKeyArn": {
                        "type": "string"
                    }
                },
                "required": [
                    "s3BucketName",
                    "s3KeyPrefix"
                ]
            }
        },
        "required": [
            "racerAlias",
            "inputModel",
            "outputMetrics",
            "outputStatus",
            "outputSimTrace",
            "outputMp4"
        ]
    },
}
