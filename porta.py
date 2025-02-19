import tkinter

def quote_generator():
    quotes = open("quotes.txt", 'r')
    print(quotes.read(5))
# encoder

user_text = input("Give me a plain text: ")
user_text2 = user_text.replace(" ", "")
key = input("Give me a 5 letter key: ")
quote_generator()




def string_split(user_text, key):
    iterations = (len(user_text) // len(key)) + 1
    newstring = ""
    position = 0
    for i in user_text:
        position += 1
        newstring = newstring + i
        if position % len(key) == 0:
            newstring = newstring + " "
    return newstring



# cipher

def porta_encrypt(message, key):
    alphabet = 'ABCDEFGHIJKLMNOPQRSTUVWXYZ'
    pairs = [
        ('A', 'B'), ('C', 'D'), ('E', 'F'), ('G', 'H'), ('I', 'J'), ('K', 'L'), 
        ('M', 'N'), ('O', 'P'), ('Q', 'R'), ('S', 'T'), ('U', 'V'), ('W', 'X'), ('Y', 'Z')
    ]
    porta_rows = [
        ('N', 'O', 'P', 'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X', 'Y', 'Z'),
        ('O', 'P', 'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X', 'Y', 'Z', 'N'),
        ('P', 'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X', 'Y', 'Z', 'N', 'O'),
        ('Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X', 'Y', 'Z', 'N', 'O', 'P'),
        ('R', 'S', 'T', 'U', 'V', 'W', 'X', 'Y', 'Z', 'N', 'O', 'P', 'Q'),
        ('S', 'T', 'U', 'V', 'W', 'X', 'Y', 'Z', 'N', 'O', 'P', 'Q', 'R'),
        ('T', 'U', 'V', 'W', 'X', 'Y', 'Z', 'N', 'O', 'P', 'Q', 'R', 'S'),
        ('U', 'V', 'W', 'X', 'Y', 'Z', 'N', 'O', 'P', 'Q', 'R', 'S', 'T'),
        ('V', 'W', 'X', 'Y', 'Z', 'N', 'O', 'P', 'Q', 'R', 'S', 'T', 'U'),
        ('W', 'X', 'Y', 'Z', 'N', 'O', 'P', 'Q', 'R', 'S', 'T', 'U', 'V'),
        ('X', 'Y', 'Z', 'N', 'O', 'P', 'Q', 'R', 'S', 'T', 'U', 'V', 'W'),
        ('Y', 'Z', 'N', 'O', 'P', 'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X'),
        ('Z', 'N', 'O', 'P', 'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X', 'Y')
    ]
    
    def pairs_match(plainletter, keyletter):
        for pair in pairs:
            if keyletter in pair:
                pair_index = pairs.index(pair)
                current_row = porta_rows[pair_index]
                if plainletter in current_row:
                    encrypted_index = current_row.index(plainletter) # ideal letter
                    return alphabet[encrypted_index]
                else:
                    encrypted_index = alphabet.index(plainletter)
                    return current_row[encrypted_index]
      
    message = message.upper()          
    key = key.upper()
    key_repeated = (key * ((len(message) // len(key)) + 1 ))[:len(message)] # multiplies key by the amount of iterations within a key, cutting off at the length of message
    
    encrypted_message = ""
    current_index = 0
    print(key_repeated)
    for i in message:
        encrypted_message += (pairs_match(i, key_repeated[current_index]))
        current_index += 1
    return encrypted_message
        
e = porta_encrypt(user_text2, key)
print(len(user_text))
print(e)
print(string_split(e, 'beach'))