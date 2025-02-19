# this tests aristocrats ig
import random

forbidden_chars = ("!", ",", ".", "?", "&", "'", '"', ";", "-", " ")
def random_alphabet():
    alphabet = "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
    alpha_list = list(alphabet)
    random.shuffle(alpha_list)
    new_alphabet = ''.join(alpha_list)
    return new_alphabet

def main():
    isEnabled = True
    while isEnabled == True:
        kill_switch = input("Generate a new encrypted text? Enter any character to start, or type exit to exit: ")
        if kill_switch.lower() == "exit":
            exit()
        else:
            new_text = input("Enter new String: ")
            new_alpha = random_alphabet()
            index = 0
            for i in new_text:
                num_val = ord(i)
                if (i in forbidden_chars):
                    pass
                else:
                    if (i.islower()):
                        new_text = new_text[:index] + new_alpha[num_val - 97].lower() + new_text[index+1:]
                    else:
                        new_text = new_text[:index] + new_alpha[num_val - 97] + new_text[index+1:]
                index += 1
            print("Encoded Text: " + new_text)
            print("Original Alphabet: ABCDEFGHIJKLMNOPQRSTUVWXYZ")
            print("Shuffled Alphabet: " + new_alpha)
main()