import speech_recognition as sr
import io

# Função que converte o áudio em texto ou repassa o comando caso já seja texto
def speech_to_text(command):
    rec = sr.Recognizer()
    mic = sr.AudioFile(command)
    try:
        with mic as source:
            audio = rec.record(source)
            return rec.recognize_google(audio,language='pt-BR')          
    except sr.RequestError as e:
        return "Erro: " +str(e)
            
    except sr.UnknownValueError:
        return "Nada foi identificado, tente novamente"