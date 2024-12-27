"use client";
import React, { useState } from 'react';

import Image from 'next/image';

import { Button } from '@/components/ui/button';
import { Input } from '@/components/ui/input';


const Login = () => {
  const [loginUser, setLoginUser] = useState('');
  const [loginPassword, setLoginPassword] = useState('');
  const [registerUser, setRegisterUser] = useState('');
  const [registerPassword, setRegisterPassword] = useState('');
  const [message, setMessage] = useState('');

  const handleLogin = async () => {
    try {
      const response = await fetch('http://127.0.0.1:8000/usuario/login', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ nome: loginUser, senha: loginPassword }),
      });

      if (response.ok) {
        const data = await response.json();
        setMessage(`Login bem-sucedido! Bem-vindo, ${data.nome}.`);
      } else {
        const error = await response.json();
        setMessage(`Erro no login: ${error.detail}`);
      }
    } catch (error) {
      setMessage('Erro ao conectar ao servidor.');
    }
  };

  const handleRegister = async () => {
    try {
      const response = await fetch(
        `http://127.0.0.1:8000/usuario/criar?id=3&nome=${registerUser}&senha=${registerPassword}&id_sessao_watson=sessao123`,
        { method: 'POST' }
      );

      if (response.ok) {
        const data = await response.json();
        setMessage(data.message);
      } else {
        setMessage('Erro ao cadastrar usuário.');
      }
    } catch (error) {
      setMessage('Erro ao conectar ao servidor.');
    }
  };
}

const Register = ({ handleLogin }) => {

  return (
    <div className="h-screen grid grid-cols-1 md:grid-cols-5 font-sans">
      {/* Seção da Esquerda */}
      <div className="col-span-2 flex flex-col items-center justify-start bg-gray-100 text-center">
        <div>
          <Image width={500} height={500} src="/logo.png" alt="logo" />
        </div>
        <div className="flex items-start flex-col">
          <h2 className="text-5xl font-bold text-[#1282A2] mb-4 text-left">
            Bem Vindo de <br /> Volta!
          </h2>
          <p className="text-3xl font-bold text-[#1282A2] mb-16">Acesse sua conta!</p>
        </div>
        <a href='/register'>
          <Button className="rounded-full py-6 w-60 h-14 flex justify-center items-center text-xl bg-[#1282A2]" >
            Cadastrar-se
          </Button>
        </a>
        <a href="#" className="mt-10 text-[#007b9f] hover:underline">
          Esqueceu a senha?
        </a>
      </div>

      {/* Seção da Direita */}
      <div className="col-span-3 flex-1 flex flex-col items-center justify-center bg-[#1282A2] text-white">
        <h2 className="text-4xl font-bold mb-8">Entre na plataforma</h2>
        <div className="flex flex-col items-center space-y-14 w-3/4">
          <Input
            type="email"
            placeholder="Email"
            className="text-black placeholder:text-gray-500 rounded-lg bg-gray-100 px-5 py-10 text-8xl h-10 border-none"
          />
          <Input
            type="password"
            placeholder="Senha"
            className="text-black placeholder:text-gray-500 rounded-lg bg-gray-100 px-5 py-10 text-8xl h-10 border-none"
          />
          <Button variant="secondary" className="rounded-full py-6 w-60 h-14 flex justify-center items-center text-xl">
            Entrar
          </Button>
        </div>
      </div>
    </div>
  );
};

export default Register;
