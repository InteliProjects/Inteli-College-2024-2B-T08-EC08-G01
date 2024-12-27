import speech_recognition as sr

rec = sr.Recognizer() 

while(1):    
    
    try:
        with sr.Microphone() as source:
            
            rec.adjust_for_ambient_noise(source, duration=0.2)
            audio = rec.listen(source)
            texto = rec.recognize_google(audio,language='pt-BR')
            texto = texto.lower()
            print("Você disse: " + texto)
            
    except sr.RequestError as e:
        print("Erro: " +str(e))
        
    except sr.UnknownValueError:
        print("Nada foi identificado")