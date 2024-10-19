

msg_str = "0:0:1:1:3:10:0:0:0:0:0:0"

def handle_rebroadcast(message_string):
    msg_list = list(message_string)

    msg_list[6] = "0" 
    new_str = "" 
 
    # traverse in the string 
    for item in msg_list: 
        new_str += item

    return new_str


if __name__ == "__main__":
   temp = handle_rebroadcast(msg_str)

   print( "0:0:1:0:3:10:0:0:0:0:0:0" == temp )