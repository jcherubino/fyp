
LOG_DICT_CONFIG = {
    'version': 1,
    'disable_existing_loggers': True,
    'formatters': {
        'standard': {
            'format': '[%(levelname)s] %(asctime)s %(name)s: %(message)s'
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
            }
        },
    'loggers': {
        '': {
            'handlers': ['default', 'file_handler'],
            'level': 'DEBUG',
        },
    }
}

