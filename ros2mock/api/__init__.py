from rosidl_runtime_py import message_to_yaml
from rosidl_runtime_py.utilities import get_service
import tempfile
import os

def edit_string_in_editor(initial_string: str) -> str:
    # Create a temporary file
    with tempfile.NamedTemporaryFile(mode='w+', delete=False, suffix='.txt') as temp_file:
        temp_file_name = temp_file.name
        # Write the initial string to the temporary file
        temp_file.write(initial_string)
        temp_file.flush()

    try:
        # Open the default editor
        editor = os.getenv('EDITOR', 'nano' if os.name != 'nt' else 'notepad')
        os.system(f"{editor} {temp_file_name}")

        # Read the modified string back from the file
        with open(temp_file_name, 'r') as temp_file:
            edited_string = temp_file.read()

    finally:
        # Clean up the temporary file
        os.unlink(temp_file_name)

    return edited_string

class ServiceResponseCompleter:
    """Callable returning a service prototype."""

    def __init__(self, *, service_type_key=None):
        self.service_type_key = service_type_key

    def __call__(self, prefix, parsed_args, **kwargs):
        service = get_service(getattr(parsed_args, self.service_type_key))
        return [message_to_yaml(service.Response())]
