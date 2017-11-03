import mmap
import sys
import os

# PYTHON VERSION
CONQUE_PYTHON_VERSION = sys.version_info[0]

if CONQUE_PYTHON_VERSION == 2:
    import cPickle as pickle
else:
    import pickle


class ConqueSoleSharedMemory():

    # is the data being stored not fixed length
    fixed_length = False

    # maximum number of bytes per character, for fixed width blocks
    char_width = 1

    # fill memory with this character when clearing and fixed_length is true
    FILL_CHAR = None

    # serialize and unserialize data automatically
    serialize = False

    # size of shared memory, in bytes / chars
    mem_size = None

    # size of shared memory, in bytes / chars
    mem_type = None

    # unique key, so multiple console instances are possible
    mem_key = None

    # mmap instance
    shm = None

    # character encoding, dammit
    encoding = 'utf-8'

    # pickle terminator
    TERMINATOR = None

    def __init__(self, mem_size = 0, mem_type = ' ', mem_key = ' ', fixed_length=False, fill_char=' ', serialize=False, encoding='utf-8'):
        """ Initialize new shared memory block instance

        Arguments:
        mem_size -- Memory size in characters, depends on encoding argument to calcuate byte size
        mem_type -- Label to identify what will be stored
        mem_key -- Unique, probably random key to identify this block
        fixed_length -- If set to true, assume the data stored will always fill the memory size
        fill_char -- Initialize memory block with this character, only really helpful with fixed_length blocks
        serialize -- Automatically serialize data passed to write. Allows storing non-byte data
        encoding -- Character encoding to use when storing character data

        """
        self.mem_size = mem_size
        self.mem_type = mem_type
        self.mem_key = mem_key
        self.fixed_length = fixed_length
        self.fill_char = fill_char
        self.serialize = serialize
        self.encoding = encoding
        self.TERMINATOR = str(chr(0)).encode(self.encoding)

        if CONQUE_PYTHON_VERSION == 3:
            self.FILL_CHAR = fill_char
        else:
            self.FILL_CHAR = unicode(fill_char)

        if fixed_length and encoding == 'utf-8':
            self.char_width = 4


    def create(self, object_path = '', file_name = '', default_value = "00000000"):
        """ Create a new block of shared memory using the mmap module. """

        BASE_PATH = "/run/obmc/sharememory"

        property_directory_path = BASE_PATH + object_path
        property_file = property_directory_path + "/" + file_name
        if not os.path.isdir(property_directory_path):
            os.makedirs(property_directory_path)

        if os.path.exists(property_file) == False:
            with open(property_file, 'w+b') as f:
                f.write(default_value)

        fd = open(property_file, "r+b")
        self.shm = mmap.mmap(fd.fileno(), 0)
        if not self.shm:
            return False
        else:
            return True


    def read(self, chars=1, start=0):
        """ Read data from shared memory.

        If this is a fixed length block, read 'chars' characters from memory.
        Otherwise read up until the TERMINATOR character (null byte).
        If this memory is serialized, unserialize it automatically.

        """
        # go to start position
        self.shm.seek(start * self.char_width)

        if self.fixed_length:
            chars = chars * self.char_width
        else:
            chars = self.shm.find(self.TERMINATOR)

        if chars == 0:
            return ''

        shm_str = self.shm.read(chars)

        # return unpickled byte object
        if self.serialize:
            return pickle.loads(shm_str)

        # decode byes in python 3
        if CONQUE_PYTHON_VERSION == 3:
            return str(shm_str, self.encoding)

        # encoding
        if self.encoding != 'ascii':
            shm_str = unicode(shm_str, self.encoding)

        return shm_str


    def write(self, text, start=0):
        """ Write data to memory.

        If memory is fixed length, simply write the 'text' characters at 'start' position.
        Otherwise write 'text' characters and append a null character.
        If memory is serializable, do so first.

        """
        # simple scenario, let pickle create bytes
        if self.serialize:
            if CONQUE_PYTHON_VERSION == 3:
                tb = pickle.dumps(text, 0)
            else:
                tb = pickle.dumps(text, 0).encode(self.encoding)

        else:
            tb = text.encode(self.encoding, 'replace')

        # write to memory
        self.shm.seek(start * self.char_width)

        if self.fixed_length:
            self.shm.write(tb)
        else:
            self.shm.write(tb + self.TERMINATOR)


    def clear(self, start=0):
        """ Clear memory block using self.fill_char. """

        self.shm.seek(start)

        if self.fixed_length:
            self.shm.write(str(self.fill_char * self.mem_size * self.char_width).encode(self.encoding))
        else:
            self.shm.write(self.TERMINATOR)

    def close(self):
        """ Close/destroy memory block. """

        self.shm.close()