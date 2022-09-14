
LOG_DICT_CONFIG = {
    'version': 1,
    'disable_existing_loggers': False,
    'formatters': {
        'standard': {
            'format': '[%(levelname)s] %(asctime)s %(name)s: %(message)s'
            },
        'data_logging': {
            'format': '%(asctime)s.%(msecs)03d,%(message)s',
            'datefmt': '%H:%M:%S',
            },
        },
    'handlers': {
        'default': {
            'level': 'DEBUG',
            'formatter': 'standard',
            'class': 'logging.StreamHandler',
        },
        'file_handler': {
            'level': 'INFO',
            'filename': 'app.log',
            'class': 'logging.handlers.RotatingFileHandler',
            'formatter': 'standard',
            'maxBytes': 2**20,
            'backupCount': 3,
            },
        'acceleration_handler': {
            'level': 'DEBUG',
            'filename': 'acceleration.log',
            'class': 'logging.FileHandler',
            'formatter': 'data_logging',
            'mode': 'a',
            },
        'position_handler': {
            'level': 'DEBUG',
            'filename': 'position.log',
            'class': 'logging.FileHandler',
            'formatter': 'data_logging',
            'mode': 'a',
            },
        },
    'loggers': {
        '': {
            'handlers': ['default', 'file_handler'],
            'level': 'DEBUG',
        },
        'accel_logger': {
            'handlers': ['acceleration_handler'],
            'level': 'DEBUG',
            'propagate': 0,
        },
        'pos_logger': {
            'handlers': ['position_handler'],
            'level': 'DEBUG',
            'propagate': 0,
        },
    }
}

